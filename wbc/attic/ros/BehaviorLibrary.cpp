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
   \file wbc/ros/BehaviorLibrary.cpp
   \author Roland Philippsen
*/

#include "BehaviorLibrary.hpp"
#include <wbc/motion/PostureBehavior.hpp>
#include <wbc/motion/FloatBehavior.hpp>
#include <wbc/parse/BehaviorParser.hpp>
#include <wbcnet/strutil.hpp>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <ros/node_handle.h>


namespace wbcros {
  
  
  BehaviorLibrary::
  BehaviorLibrary(std::string const & param_prefix)
    : behaviors_param_name_(param_prefix + "behaviors")
  {
  }
  
  
  BehaviorLibrary::
  ~BehaviorLibrary()
  {
    for (size_t ii(0); ii < behavior_.size(); ++ii) {
      delete behavior_[ii];
    }
  }
  
  
  void BehaviorLibrary::
  initDummy(Model & model)
  {
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
    behavior_.back()->robotControlModel(model.control_model_);
  }
  
  
  void BehaviorLibrary::
  initFromParam(Model & model, ros::NodeHandle &nn,
		wbc::BehaviorFactoryRegistry const & breg) throw(std::runtime_error)
  {
    XmlRpc::XmlRpcValue bval;
    if ( ! nn.getParam(behaviors_param_name_, bval)) {
      throw std::runtime_error("wbcros::BehaviorLibrary::initFromParam(): invalid behaviors_param_name_ \""
			       + behaviors_param_name_ + "\"");
    }
    
    try {
      wbc::StdBehaviorConstructionCallback bcc(behavior_, breg);
      for (XmlRpc::XmlRpcValue::iterator ib(bval.begin()); ib != bval.end(); ++ib) {
	wbc::BehaviorConstructionCallback::dictionary_t bparams;
	bparams.insert(make_pair("type", ib->first));
	for (int ip(0); ip < ib->second.size(); ++ip) {
	  string key, value;
	  sfl::splitstring(ib->second[ip], ' ', key, value);
	  bparams.insert(make_pair(key, value));
	}
	bcc(bparams);
	ROS_INFO ("wbcros::BehaviorLibrary::initFromParam(): added %s", ib->first.c_str());
      }
    }
    catch (XmlRpc::XmlRpcException const & ee) {
      std::ostringstream msg;
      msg << "wbcros::BehaviorLibrary::initFromParam():"
	  << " XmlRpcException while reading behaviors: "
	  << ee.getMessage();
      throw std::runtime_error(msg.str());
    }
    
    if (behavior_.empty()) {
      throw std::runtime_error("wbcros::BehaviorLibrary::initFromParam(): no behaviors specified");
    }
    
    for (size_t ii(0); ii < behavior_.size(); ++ii) {
      behavior_[ii]->robotControlModel(model.control_model_);
      ROS_INFO ("wbcros::BehaviorLibrary::initFromParam(): initialized %s", behavior_[ii]->name.c_str());
    }
  }
  
}
