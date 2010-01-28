/*
 * ROS support for Stanford-WBC http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file wbc/ros_support/BehaviorLibrary.cpp
   \author Roland Philippsen
*/

#include "BehaviorLibrary.hpp"
#include <wbc/motion/PostureBehavior.hpp>
#include <wbc/motion/FloatBehavior.hpp>

namespace wbcros {
  
  
  BehaviorLibrary::
  ~BehaviorLibrary()
  {
    for (size_t ii(0); ii < behavior_.size(); ++ii) {
      delete behavior_[ii];
    }
  }
  
  
  void BehaviorLibrary::
  initDefault(Model & model)
  {
    // should use behavior factories from WBC plugins, or some other
    // runtime configuration mechanism
    wbc::PostureBehavior * posture_behavior(new wbc::PostureBehavior());
    posture_behavior->robotControlModel(model.control_model_);
    posture_behavior->setFloatKey('f');
    posture_behavior->setFreezeKey(' ');
    SAIVector posture(model.ndof_actuated_);
    double const dtheta(10 * M_PI / 180);
    posture.zero();
    posture[0] = dtheta;
    posture_behavior->addPostureKey('0', posture);
    if (model.ndof_actuated_ > 1) {
      posture.zero();
      posture[1] = dtheta;
      posture_behavior->addPostureKey('1', posture);
    }
    if (model.ndof_actuated_ > 2) {
      posture.zero();
      posture[2] = dtheta;
      posture_behavior->addPostureKey('2', posture);
    }
    if (model.ndof_actuated_ > 3) {
      posture.zero();
      posture[3] = dtheta;
      posture_behavior->addPostureKey('3', posture);
    }
    if (model.ndof_actuated_ > 4) {
      posture.zero();
      posture[4] = dtheta;
      posture_behavior->addPostureKey('4', posture);
    }
    if (model.ndof_actuated_ > 5) {
      posture.zero();
      posture[5] = dtheta;
      posture_behavior->addPostureKey('5', posture);
    }
    if (model.ndof_actuated_ > 6) {
      posture.zero();
      posture[6] = dtheta;
      posture_behavior->addPostureKey('6', posture);
    }
    if (model.ndof_actuated_ > 7) {
      posture.zero();
      posture[7] = dtheta;
      posture_behavior->addPostureKey('7', posture);
    }
    if (model.ndof_actuated_ > 8) {
      posture.zero();
      posture[8] = dtheta;
      posture_behavior->addPostureKey('8', posture);
    }
    if (model.ndof_actuated_ > 9) {
      posture.zero();
      posture[9] = dtheta;
      posture_behavior->addPostureKey('9', posture);
    }
    behavior_.push_back(posture_behavior);
    behavior_.push_back(new wbc::FloatBehavior());
  }
  
}
