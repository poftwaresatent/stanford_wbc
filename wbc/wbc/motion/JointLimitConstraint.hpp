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
  \author     Duong (John) Dang
  \file       JointLimitConstraint.hpp
*/
//===========================================================================

#ifndef WBC_JOINT_LIMIT_CONSTRAINT_HPP
#define WBC_JOINT_LIMIT_CONSTRAINT_HPP

#include <wbc/core/TaskDescription.hpp>
#include <wbc/core/BranchingRepresentation.hpp>

struct timeval;

namespace wbc {


  class JointLimitConstraint : public TaskDescription {
  public:

    explicit JointLimitConstraint(std::string const & name);
  
    virtual const SAIVector& commandAccel() const { return commandAccel_; }
    virtual const SAIMatrix& Jacobian() const { return globalJacobian_; }
  
    virtual void onUpdate();

    virtual void robotControlModel( RobotControlModel* ) throw(std::runtime_error);
    virtual void reset();
    
  private:

    // references
    RobotControlModel* robModel_;
     
    //! Attributes.
  
    SAIMatrix globalJacobian_;
    int dimension_;// = number of violated constraint
    Float hysteresis_; 
    SAIVector velocity_;
    SAIVector commandAccel_;
    SAIVector position_;
    SAIVector upperLimit_;
    SAIVector lowerLimit_;
    double safetyDistance_;
    int ndof_;

    //! Private functions.
    void calculateJacobian();
    void servoUpdate();
    void calculateState();
    void calculateViolatingLimits();
    SAIVector limitFlags;
    SAIVector jointViolatingLimit_;
    timeval* startTime;
    SAIVector a0,a1,a2,a3;
    Float cubicDuration;


    //! Friends.
    // friend class WholeBodyControllerImpl;
    //friend class TaskDescription; 
	

  };

}

#endif
