/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

//===========================================================================
/*!
  \file JointLimitConstraint.cpp
  \author Duong (John) Dang
*/
//===========================================================================

#include <wbc/motion/JointLimitConstraint.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/Kinematics.hpp>
#include <iostream>

namespace wbc {

  static bool computeCubicSpline(const Float &a0,
				 const Float &a1,
				 const Float &a2,
				 const Float &a3,
				 const Float &endTime,
				 const Float &t,
				 Float &x,
				 Float &dx,
				 Float&ddx 
				 );
  
  static void computeCubicConstant(const Float &startPos,
				   const Float &endPos,
				   const Float &startVel,
				   const Float &endTime,
				   Float &a0,
				   Float &a1,
				   Float &a2,
				   Float &a3  
				   );
  
  JointLimitConstraint::JointLimitConstraint(std::string const & name)
    : TaskDescription(name, 100, 50, -1, -1),
      hysteresis_(0.1)
  {
  }


  void
  JointLimitConstraint::reset(){
    jointViolatingLimit_=SAIVector(ndof_);
    limitFlags=SAIVector(ndof_);
    globalJacobian_= SAIMatrix(0,ndof_);
    commandAccel_= SAIVector(0);
    dimension_=0;
    a0=SAIVector(ndof_);
    a1=SAIVector(ndof_);
    a2=SAIVector(ndof_);
    a3=SAIVector(ndof_);
    cubicDuration=0.1;
  }
  
  
  void
  JointLimitConstraint::robotControlModel( RobotControlModel* robModel )
    throw(std::runtime_error)
  {
    robModel_ = robModel;
    ndof_=robModel_->branching()->numJoints();
    upperLimit_=robModel_->branching()->upperJointLimits();
    lowerLimit_=robModel_->branching()->lowerJointLimits();
    //  upperLimit_.display("upper_limit");
    //  lowerLimit_.display("lower_limit");
    jointViolatingLimit_=SAIVector(ndof_);
    limitFlags=SAIVector(ndof_);
    globalJacobian_=SAIMatrix(0,ndof_);
    startTime = new timeval[ndof_];
    commandAccel_=SAIVector(0);
    dimension_=0;
    a0=SAIVector(ndof_);
    a1=SAIVector(ndof_);
    a2=SAIVector(ndof_);
    a3=SAIVector(ndof_);
    cubicDuration=0.1;
  }
  
  
  void
  JointLimitConstraint::onUpdate() { 
    //check for joint limits Violation
    calculateViolatingLimits();
    //compute Jacobian
    calculateJacobian();
    //update config_ and config
    servoUpdate();
    //cout << "dimension_= " << dimension_ <<endl;
  } 
  
  
  void 
  JointLimitConstraint::servoUpdate() { 
    commandAccel_=SAIVector(dimension_);
    timeval curTime=robModel_->time();
    if (dimension_==0)
      return;
    int j=0;
    const SAIVector& jointPos( robModel_->kinematics()->jointPositions() );
    const SAIVector& jointVel( robModel_->kinematics()->jointVelocities() );

    for (int i=0;i<ndof_;i++)
      {
	if (limitFlags[i]!=0) 
	  {
	    Float goalPos,goalVel,goalAcc;
	    Float timej
	      = (curTime.tv_usec - startTime[i].tv_usec)/1e6
	      + (curTime.tv_sec - startTime[i].tv_sec)  ;
	    computeCubicSpline(a0[i],a1[i],a2[i],a3[i],
			       cubicDuration,timej,goalPos,goalVel,goalAcc);
	    commandAccel_[j]
	      = goalAcc + m_diffGain*(goalVel-jointVel[i]) + m_propGain*(goalPos-jointPos[i]);
	    j++;
	  }
      }	
  }
  
  
  void
  JointLimitConstraint::calculateViolatingLimits() {
    const SAIVector& jointPos( robModel_->kinematics()->jointPositions() );
    const SAIVector& jointVel( robModel_->kinematics()->jointVelocities() );
  
    timeval curTime=robModel_->time();
    jointViolatingLimit_.zero();
    for( int i = 0; i < ndof_ ; i++ ) {
      // if the flag is already set
      if (limitFlags[i]==0){
	if ( jointPos[i] <=lowerLimit_[i] + hysteresis_){
	  limitFlags[i]=-1;
	  jointViolatingLimit_[i]=abs(jointPos[i]-lowerLimit_[i]);
	  dimension_++;
	  startTime[i]=curTime;
	  cout << "Blocking joint " << i + 1
	       << ". Now total blocked joints = "<< dimension_ << endl;
	  computeCubicConstant(jointPos[i],
			       lowerLimit_[i]+hysteresis_/2,
			       jointVel[i],
			       cubicDuration,
			       a0[i],a1[i],a2[i],a3[i]);
	} 
	else if ( jointPos[i] >=upperLimit_[i] - hysteresis_){
	  limitFlags[i]=1;
	  jointViolatingLimit_[i]=abs(jointPos[i]-upperLimit_[i]);
	  dimension_++;
	  startTime[i]=curTime;
	  computeCubicConstant(jointPos[i],
			       upperLimit_[i]-hysteresis_/2,
			       jointVel[i],
			       cubicDuration,
			       a0[i],a1[i],a2[i],a3[i]);
	  cout << "Blocking joint " << i + 1
	       << ". Now total blocked joints = "<< dimension_ <<  endl;
	}
      }
	
    }

  }


  void
  JointLimitConstraint::calculateJacobian() {
    int k = 0;
    if( dimension_ == 0 ){
      // if no violation: create null Jacobian matrix	  
      globalJacobian_ = SAIMatrix(0,ndof_);
      return;
    }
    globalJacobian_.setSize( dimension_, ndof_, true );
    //cout << "Dimension=" << dimension_ << "\n";
    for( int i = 0; i < ndof_; i++ ) {
      if( limitFlags[i] != 0 ) {
	// cout << "Joint " << i << "'s limit violated\n";	    
	// set 1 to the col where the joint limit is violated
	globalJacobian_.elementAt(k,i) = 1.0;
	k++;
      }
    }
  }
  
  
  bool computeCubicSpline(const Float &a0,
			  const Float &a1,
			  const Float &a2,
			  const Float &a3,
			  const Float &endTime ,
			  const Float &t,
			  Float &x,
			  Float &dx,
			  Float&ddx 
			  )
  {
    if (t>endTime){
      x=a0+a1*endTime+a2*endTime*endTime+a3*endTime*endTime*endTime;
      dx=0;
      //		ddx=0;
      return true;
    }
    x=a0+a1*t+a2*t*t+a3*t*t*t;
    dx=  a1+2*a2*t+3*a3*t*t;
    ddx= 2*a2 + 6*a3*t;
    return true;
  }
  
  
  void computeCubicConstant(const Float &startPos,
			    const Float &endPos,
			    const Float &startVel,
			    const Float &endTime,
			    Float &a0,
			    Float &a1,
			    Float &a2,
			    Float &a3
			    )
  {
    Float T=endTime;
    a0=startPos;
    a1=startVel;
    a2=(3*endPos- 2*a1*T-3*a0)/(T*T);
    a3=(-2*endPos+ a1*T+2*a0)/(T*T*T);
  }

}
