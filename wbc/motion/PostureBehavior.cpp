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

//=========================================================================
/*!
  \author     Luis Sentis
  \file       PostureBehavior.cpp
*/
//=========================================================================

#include <wbc/motion/PostureBehavior.hpp>
#include <wbc/core/TaskSet.hpp>
#include <wbc/motion/WholeBodyPosture.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/util/Recorder.hpp>
#include <saimatrix/SAIVector.h>
#include <wbcrun/service.hpp>

namespace wbc {

  enum {
    FLOAT,
    ACTIVE
  } ;

  void
  PostureBehavior::onUpdate() {

    whole_body_posture_.onUpdate();
    friction_posture_.onUpdate();

    SAIVector defaultPosture = robModel()->branching()->defaultJointPositions();
    //defaultPosture.display("desired posture");
    //robModel()->kinematics()->jointPositions().display("actual posture");


    //==============
    // state machine
    //==============

    switch( currentState_ ) {

      // Move hands to position
    case ACTIVE:
      whole_body_posture_.goalPostureConfig(goalPosture_);
      activeTaskSet_ = &taskSetOperational_;
      break;
    case FLOAT:
      activeTaskSet_ = &taskSetFloat_;
      break;

    default:
      break;
    }
  }


  void
  PostureBehavior::loadMovementPrimitives( RobotControlModel* robmodel )
    throw(std::runtime_error)
  {
    // pass robot model to task primitives 
    whole_body_posture_.robotControlModel( robmodel );
    friction_posture_.robotControlModel( robmodel );

    // Reset task set.
    taskSetOperational_.removeAll();
    taskSetFloat_.removeAll();
    taskSetFloat_.addTask(&friction_posture_);
    friction_posture_.diffGain(1.0);
    //=============================================//
    // BEGIN LOAD TASK SET MOVE HAND TO POSITION
    //=============================================//
    taskSetOperational_.addTask( &whole_body_posture_ );


    //==================//
    // REGISTER TASK SETS WITH BASE CLASS
    //==================//
    registerTaskSet(&taskSetOperational_);
    registerTaskSet(&taskSetFloat_);
    //==================//
    // INITIALIZATIONS
    //==================//

    // whole-body posture
    SAIVector defaultPosture = robModel()->branching()->defaultJointPositions();
    goalPosture_ = defaultPosture;
    whole_body_posture_.goalPostureConfig(goalPosture_);
    whole_body_posture_.propGain(400.0);
    whole_body_posture_.diffGain(15.0);
    whole_body_posture_.maxVel(0.85);//rad/s

    // activate task set
    activeTaskSet_ = &taskSetOperational_;
  }


  PostureBehavior::PostureBehavior()
    : BehaviorDescription("PostureBehavior"),
      whole_body_posture_("whole_body_posture"),
      friction_posture_("friction_posture"),
      currentState_(0) {} 

  int32_t PostureBehavior::
  handleCommand(int32_t const * codeVector,
		size_t nCodes,
		SAIMatrix const & matrix)
  {
    switch (codeVector[0]) {      
    case wbcrun::srv::SET_GOAL:
      cout << "recieved SET_GOAL command" << endl;
      if (matrix.column()!=1 || matrix.row()!=robModel()->branching()->numJoints()){
	cout << "wrong dimension: " << matrix.column() << "  " << matrix.row() << endl;
	return wbcrun::srv::INVALID_DIMENSION;
      }
      else {
	for (int i = 0; i< robModel()->branching()->numJoints(); i++)
	  goalPosture_[i] = matrix[0][i];
	return wbcrun::srv::SUCCESS;
	break;
      }

    case wbcrun::srv::FLOAT:
      cout << "recieved FLOAT command" << endl;
      currentState_ = FLOAT;
      return wbcrun::srv::SUCCESS;
      break;
  

    case wbcrun::srv::ACTIVATE:
      cout << "recieved FLOAT command" << endl;
      goalPosture_ = robModel()->kinematics()->jointPositions();
      currentState_ = ACTIVE;
      return wbcrun::srv::SUCCESS;
      break;


    default:
      return wbcrun::srv::NOT_IMPLEMENTED;
    }

  }

}
