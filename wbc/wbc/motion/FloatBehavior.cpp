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
  \file       FloatBehavior.cpp
  \author     Luis Sentis
*/
//=========================================================================

#include <wbc/motion/FloatBehavior.hpp>
#include <wbc/core/TaskSet.hpp>
#include <saimatrix/SAIVector.h>
#include <wbc/motion/PositionTask.hpp>
#include <wbc/motion/OrientationTask.hpp>
#include <wbc/motion/WholeBodyPosture.hpp>

namespace wbc {

  void
  FloatBehavior::onUpdate() {

    friction_posture_.onUpdate();

    //==============
    // state machine
    //==============

    switch( currentState_ ) {

      // Move hands to position
    case 0:
      break;
    default:
      break;
    }
  }

  void
  FloatBehavior::loadMovementPrimitives( RobotControlModel* robmodel ) throw(std::runtime_error) {

    friction_posture_.robotControlModel( robmodel );

    // Reset task set.
    taskSet_.removeAll();


    //=============================================//
    // BEGIN LOAD TASK SET MOVE HAND TO POSITION
    //=============================================//

    taskSet_.addTask( &friction_posture_ );

    //==================//
    // REGISTER TASK SETS WITH BASE CLASS
    //==================//
    registerTaskSet(&taskSet_);
  
    //==================//
    // INITIALIZATIONS
    //==================//

    friction_posture_.diffGain(11.0);

    // activate task set
    activeTaskSet_ = &taskSet_;
  }

  FloatBehavior::FloatBehavior()
    : BehaviorDescription("FloatBehavior"),
      friction_posture_("friction_posture"),
      currentState_(0) {} 

}
