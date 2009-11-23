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
  \author     Luis Sentis
  \file       PositionTask.cpp 
*/
//===========================================================================

#include <wbc/motion/PositionTask.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/Kinematics.hpp>
#include <saimatrix/SAITransform.h>
#include <iostream> 

namespace wbc {

  PositionTask::PositionTask(std::string const & name,
			     std::string const & linkName) 
    : TaskDescription(name, 400, 40, 0.1, 0),
      commandAccel_(3),
      propGainVec_(0),
      diffGainVec_(0),
      linkName_(linkName),
      link_(0),
      selectionMatrix_(3,3) 
  {

    selectionMatrix_.identity();
  }


  void 
  PositionTask::directionSelection( bool xdir, bool ydir, bool zdir ) {
  
    int i = 0;
  
    if (xdir) i++;
    if (ydir) i++;
    if (zdir) i++;
  
    selectionMatrix_.setSize(i, 3, true);
  
    i = 0;
  
    if (xdir){
      selectionMatrix_.elementAt(i,0) = 1;
      i++ ;
    }

    if (ydir){
      selectionMatrix_.elementAt(i,1) = 1;
      i++ ;
    }

    if (zdir){
      selectionMatrix_.elementAt(i,2) = 1;
      i++ ;
    }
  }


  void PositionTask::onUpdate() {

    // update position
    config_ = robModel_->kinematics()->globalFrame( link_, SAIVector(0) ).translation();
  
    // update jacobian
    globalJacobian_ = robModel_->kinematics()->JacobianAtPoint( link_, SAIVector(0) );
    int col = globalJacobian_.column();
  
    globalJacobian_ = globalJacobian_.submatrix( 0, 0, 3, col );

    // update velocity
    velocity_ = globalJacobian_ * robModel_->kinematics()->jointVelocities();

    globalJacobian_ = selectionMatrix_ * globalJacobian_;


    // update servo
    servoUpdate();
  }


  void PositionTask::servoUpdate() {

    SAIVector desDiff(3);
    double satFactor, satFactor2, inv_rate;;

    if( propGainVec_.size() > 0 && diffGainVec_.size() > 0 ) {

      ////int foo = 0;
      for( int ii(0); ii < 3; ++ii ) {
	desDiff[ii] = ( goalConfig_[ii] -  config_[ii] ) * propGainVec_[ii];

	// Saturating upon velocity limits
	if( diffGainVec_[ii] != 0 ) {

	  satFactor = fabs( desDiff[ii] ) / ( m_maxVel * diffGainVec_[ii] );
	  if( satFactor > 1 ) desDiff[ii] *= 1/satFactor;
	}
    
	// desiredAccel_ = -m_diffGain (diffState_ - \nu desDiff_ );
	commandAccel_[ii] = desDiff[ii] - velocity_[ii] * diffGainVec_[ii]; 
      }
    }
    else {

      // desDiff = -( state_ - goalState_ ) * gain_
      desDiff = ( goalConfig_ -  config_ ) * m_propGain;
    
      // Saturating upon velocity limits
      if( m_diffGain != 0 ) {
	satFactor = desDiff.abs() / ( m_maxVel * m_diffGain );
	if( satFactor > 1 ) desDiff.multiply( 1/satFactor );
      }
    
      // desiredAccel_ = -m_diffGain (diffState_ - \nu desDiff_ );
      //cout << "desdiff size: " << desDiff.size() << "velocity size: " << velocity_.size() << std::endl;
      commandAccel_ = desDiff - velocity_ * m_diffGain; 
    
      // Saturation of acceleration.
      if( fabs(m_maxAccel) > 1e-2 ) { // this is to ensure that acceleration saturation has been set up
	inv_rate =  commandAccel_.abs() / m_maxAccel ;
	if( inv_rate > 1 ) satFactor2 = 1.0 / inv_rate;
	else satFactor2 = 1.0;
	commandAccel_ *= satFactor2;
      } 
    
      commandAccel_ = selectionMatrix_ * commandAccel_;
    }
  }
  
  
  void PositionTask::
  robotControlModel( RobotControlModel * robmodel )
    throw(std::runtime_error)
  {
    robModel_ = robmodel;
    link_ = robmodel->branching()->findLink(linkName_);
    if ( ! link_)
      throw std::runtime_error("wbc::PositionTask::robotControlModel(): no link with name `"
			       + linkName_ + "'");
  }
  
}
