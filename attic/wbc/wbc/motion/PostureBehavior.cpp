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
#include <wbcnet/strutil.hpp>
#include <sstream>

#ifdef WIN32
__inline const int rint(float xx) { return (xx+0.5); }
#endif // WIN32

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {
  
  
  PostureBehavior::
  PostureBehavior()
    : BehaviorDescription("PostureBehavior"),
      float_key_(0),
      freeze_key_(0),
      whole_body_posture_("whole_body_posture"),
      friction_posture_("friction_posture"),
      freeze_requested_(true)
  {
  }
  
  
  void PostureBehavior::
  onUpdate()
  {
    if (freeze_requested_) {
      LOG_INFO (logger, "wbc::PostureBehavior::onUpdate(): switching to OPERATIONAL task set (and FREEZE)");
      whole_body_posture_.goalPostureConfig(robModel()->kinematics()->jointPositions());
      activeTaskSet_ = &taskSetOperational_;
      freeze_requested_ = false;
    }
    whole_body_posture_.onUpdate();
    friction_posture_.onUpdate();
  }
  
  
  void PostureBehavior::
  reset()
  {
    freeze_requested_ = true;
  }
  
  
  void PostureBehavior::
  loadMovementPrimitives(RobotControlModel * robmodel)
    throw(std::runtime_error)
  {
    taskSetOperational_.removeAll();
    
    whole_body_posture_.robotControlModel(robmodel);
    whole_body_posture_.goalPostureConfig(robModel()->branching()->defaultJointPositions());
    whole_body_posture_.propGain(100.0);
    whole_body_posture_.diffGain(0.0);
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
  
  
  int PostureBehavior::
  handleSetGoal(SAIVector const & goal)
  {
    if (goal.size() != robModel()->branching()->numJoints()) {
      return wbcnet::SRV_INVALID_DIMENSION;
    }
    whole_body_posture_.goalPostureConfig(goal);
    activeTaskSet_ = &taskSetOperational_;
    return wbcnet::SRV_SUCCESS;
  }
  
  
  int PostureBehavior::
  handleSetGains(SAIVector const & gains)
  {
    if (2 > gains.size()) {
      return wbcnet::SRV_INVALID_DATA;
    }
    if (0 > gains[1]) {
      return wbcnet::SRV_INVALID_DATA;
    }
    if ((2 < gains.size()) && (0 > gains[2])) {
      return wbcnet::SRV_INVALID_DATA;
    }
    
    int const ndof(robModel()->branching()->numJoints());
    int index(static_cast<int>(rint(gains[0])));
    bool reset_others(false);
    if (index < 0) {
      reset_others = true;
      index = -index;
    }
    
    if (index >= ndof) {
      return wbcnet::SRV_INVALID_DIMENSION;
    }
    
    if (whole_body_posture_.test_kp.size() != ndof) {
      int const oldsize(whole_body_posture_.test_kp.size());
      whole_body_posture_.test_kp.setSize(ndof, false);
      for (int ii(oldsize); ii < ndof; ++ii) {
	whole_body_posture_.test_kp[ii] = 0;
      }
    }
    
    if (whole_body_posture_.test_kd.size() != ndof) {
      int const oldsize(whole_body_posture_.test_kd.size());
      whole_body_posture_.test_kd.setSize(ndof, false);
      for (int ii(oldsize); ii < ndof; ++ii) {
	whole_body_posture_.test_kd[ii] = 0;
      }
    }
    
    if (reset_others) {
      whole_body_posture_.test_kp.zero();
    }
    whole_body_posture_.test_kp[index] = gains[1];
    
    if (gains.size() > 2) {
      if (reset_others) {
	whole_body_posture_.test_kd.zero();
      }
      whole_body_posture_.test_kd[index] = gains[2];
    }
    else {
      whole_body_posture_.test_kd[index] = 0;
    }
    
    return wbcnet::SRV_SUCCESS;
  }
  
  
  int PostureBehavior::
  handleKey(int keycode)
  {
    LOG_INFO (logger, "wbc::PostureBehavior::handleKey(): keycode = " << keycode);
    
    if ((0 != float_key_) && (keycode == float_key_)) {
      LOG_INFO (logger, "wbc::PostureBehavior::handleKey(): switching to FLOAT task set");
      activeTaskSet_ = &taskSetFloat_;
      return wbcnet::SRV_SUCCESS;
    }
    
    if ((0 != freeze_key_) && (keycode == freeze_key_)) {
      freeze_requested_ = true;
      return wbcnet::SRV_SUCCESS;
    }
    
    key_posture_t::iterator iposture(key_posture_.find(keycode));
    if (key_posture_.end() == iposture) {
      LOG_WARN (logger, "wbc::PostureBehavior::handleKey(): unrecognized keycode");
      //       std::cerr << "wbc::PostureBehavior::handleKey(): unrecognized keycode " << keycode << "\n";
      //       for (key_posture_t::const_iterator ii(key_posture_.begin()); ii != key_posture_.end(); ++ii) {
      // 	std::cerr << "  " << ii->first << " : " << ii->second << "\n";
      //       }
      return wbcnet::SRV_OTHER_ERROR;
    }
    
    if (iposture->second.size() != robModel()->branching()->numActuatedJoints()) {
      static bool const strict(false);
      if (strict) {
	LOG_ERROR (logger,
		   "wbc::PostureBehavior::handleKey(): posture for key code " << keycode
		   << " has invalid size " << iposture->second.size()
		   << " (should be " << robModel()->branching()->numActuatedJoints() << ")");
	return wbcnet::SRV_INVALID_DIMENSION;
      }
      int const oldsize(iposture->second.size());
      int const newsize(robModel()->branching()->numActuatedJoints());
      iposture->second.setSize(newsize, false);
      for (int ii(oldsize); ii < newsize; ++ii) {
	iposture->second[ii] = 0;
      }
    }
    
    LOG_INFO (logger, "wbc::PostureBehavior::handleKey(): switching to posture " << keycode);
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
  
  
  bool PostureBehavior::
  handleInit(std::string const & key, std::string const & value) throw(std::runtime_error)
  {
    if ("posture_key" == key) {
      istringstream is(value);
      if ( ! is) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): gimme a key and some numbers!");
      }
      char keycode;
      is >> keycode;
      vector<double> jpos;
      while (is) {
	double foo;
	is >> foo;
	if (is) {
	  jpos.push_back(foo);
	}
      }
      addPostureKey(keycode, SAIVector(&jpos[0], jpos.size()));
      LOG_INFO (logger, "wbc::PostureBehavior::handleInit(): registered keycode " << (int) keycode);
      return true;
    }
    
    else if ("float_key" == key) {
      istringstream is(value);
      if ( ! is) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): gimme a key!");
      }
      char keycode;
      is >> keycode;
      setFloatKey(keycode);
      LOG_INFO (logger, "wbc::PostureBehavior::handleInit(): float_key set to " << (int) keycode);
      return true;
    }
    
    else if ("freeze_key" == key) {
      istringstream is(value);
      if ( ! is) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): gimme a key!");
      }
      char keycode;
      is >> keycode;
      setFreezeKey(keycode);
      LOG_INFO (logger, "wbc::PostureBehavior::handleInit(): freeze_key set to " << (int) keycode);
      return true;
    }
    
    else if ("whole_body_prop_gain" == key) {
      istringstream is(value);
      double kp;
      is >> kp;
      if ( ! is) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): kp expected");
      }
      if (0 > kp) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): invalid kp " + sfl::to_string(kp) + " (must be >= zero)");
      }
      whole_body_posture_.propGain(kp);
      LOG_INFO (logger, "wbc::PostureBehavior::handleInit(): kp set to " << kp);
      return true;
    }
    
    else if ("whole_body_diff_gain" == key) {
      istringstream is(value);
      double kv;
      is >> kv;
      if ( ! is) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): kv expected");
      }
      if (0 > kv) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): invalid kv " + sfl::to_string(kv) + " (must be >= zero)");
      }
      whole_body_posture_.diffGain(kv);
      LOG_INFO (logger, "wbc::PostureBehavior::handleInit(): kv set to " << kv);
      return true;
    }
    
    else if ("whole_body_max_vel" == key) {
      istringstream is(value);
      double vmax;
      is >> vmax;
      if ( ! is) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): vmax expected");
      }
      if (0 > vmax) {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): invalid vmax " + sfl::to_string(vmax) + " (must be >= zero)");
      }
      whole_body_posture_.maxVel(vmax);
      LOG_INFO (logger, "wbc::PostureBehavior::handleInit(): vmax set to " << vmax);
      return true;
    }
    
    else if ("whole_body_enable_vsat" == key) {
      if ("true" == value) {
	whole_body_posture_.enable_vsat_ = true;
      }
      else if ("false" == value) {
	whole_body_posture_.enable_vsat_ = false;
      }
      else {
	throw runtime_error("wbc::PostureBehavior::handleInit(" + key + ", " + value
			    + "): expected 'true' or 'false'");
      }
      LOG_INFO (logger, "wbc::PostureBehavior::handleInit(): enable_vsat set to " << value);
      return true;
    }
    
    return false;
  }
  
}
