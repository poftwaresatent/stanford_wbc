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

/**
  \file PostureBehavior.cpp
  \author Luis Sentis and Roland Philippsen
*/

#include <wbc/motion/PostureBehavior.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbcnet/msg/Service.hpp>
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {
  
  
  PostureBehavior::
  PostureBehavior()
    : BehaviorDescription("PostureBehavior"),
      float_key_(0),
      freeze_key_(0),
      whole_body_posture_("whole_body_posture"),
      friction_posture_("friction_posture")
  {
  }
  
  
  void PostureBehavior::
  onUpdate()
  {
    whole_body_posture_.onUpdate();
    friction_posture_.onUpdate();
  }


  void PostureBehavior::
  loadMovementPrimitives(RobotControlModel * robmodel)
    throw(std::runtime_error)
  {
    taskSetOperational_.removeAll();
    
    whole_body_posture_.robotControlModel(robmodel);
    whole_body_posture_.goalPostureConfig(robModel()->branching()->defaultJointPositions());
    whole_body_posture_.propGain(400.0);
    whole_body_posture_.diffGain(15.0);
    whole_body_posture_.maxVel(0.85);//rad/s
    taskSetOperational_.addTask(&whole_body_posture_);
    registerTaskSet(&taskSetOperational_);
    
    friction_posture_.robotControlModel(robmodel);
    friction_posture_.diffGain(1.0);
    taskSetFloat_.removeAll();
    taskSetFloat_.addTask(&friction_posture_);
    registerTaskSet(&taskSetFloat_);
    
    activeTaskSet_ = &taskSetOperational_;
  }
  
  
  int32_t PostureBehavior::
  handleCommand(int32_t const * codeVector,
		size_t nCodes,
		SAIMatrix const & matrix)
  {
    // handleStdCommand() will dispatch e.g. to handleKey() as
    // appropriate by looking at the codeVector
    
    int32_t const std_result(handleStdCommand(codeVector, nCodes, matrix));
    if (wbcnet::SRV_NOT_IMPLEMENTED != std_result) {
      return std_result;
    }
    
    // handleStdCommand() did not understand the command, let's see
    // what we can do with it...
    
    // this switch has only one case (at the moment), which looks
    // weird, but we hope to extend this "soon"
    switch (codeVector[0]) {      
    
    case wbcnet::SRV_SET_GOAL:
      cout << "recieved SET_GOAL command" << endl;
      if (matrix.column()!=1 || matrix.row()!=robModel()->branching()->numJoints()){
	cout << "wrong dimension: " << matrix.column() << "  " << matrix.row() << endl;
	return wbcnet::SRV_INVALID_DIMENSION;
      }
      else {
	SAIVector goalPosture(robModel()->branching()->numJoints());
	for (int i = 0; i< robModel()->branching()->numJoints(); i++)
	  goalPosture[i] = matrix[0][i];
	whole_body_posture_.goalPostureConfig(goalPosture);
	activeTaskSet_ = &taskSetOperational_;
	return wbcnet::SRV_SUCCESS;
      }

    }
    
    return wbcnet::SRV_NOT_IMPLEMENTED;
  }
  
  
  int32_t PostureBehavior::
  handleKey(int32_t keycode)
  {
    if ((0 != float_key_) && (keycode == float_key_)) {
      activeTaskSet_ = &taskSetFloat_;
      return wbcnet::SRV_SUCCESS;
    }
    
    if ((0 != freeze_key_) && (keycode == freeze_key_)) {
      whole_body_posture_.goalPostureConfig(robModel()->kinematics()->jointPositions());
      activeTaskSet_ = &taskSetOperational_;
      return wbcnet::SRV_SUCCESS;
    }
    
    key_posture_t::const_iterator iposture(key_posture_.find(keycode));
    if (key_posture_.end() == iposture) {
      return wbcnet::SRV_NOT_IMPLEMENTED;
    }
    
    if (iposture->second.size() != robModel()->branching()->numActuatedJoints()) {
      LOG_ERROR (logger,
		 "wbc::PostureBehavior::handleKey(): posture for key code " << keycode
		 << " has invalid size " << iposture->second.size()
		 << " (should be " << robModel()->branching()->numActuatedJoints() << ")");
      return wbcnet::SRV_INVALID_DIMENSION;
    }
    
    whole_body_posture_.goalPostureConfig(iposture->second);
    activeTaskSet_ = &taskSetOperational_;
    return wbcnet::SRV_SUCCESS;
  }
  
  
  void PostureBehavior::
  setFloatKey(int32_t keycode)
  {
    float_key_ = keycode;
  }
  
  
  void PostureBehavior::
  setFreezeKey(int32_t keycode)
  {
    freeze_key_ = keycode;
  }
  
  
  void PostureBehavior::
  addPostureKey(int32_t keycode, SAIVector const & posture)
  {
    key_posture_[keycode] = posture;
  }
  
}
