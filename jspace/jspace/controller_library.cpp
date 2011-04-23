/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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
   \file jspace/controller_library.cpp
   \author Roland Philippsen
*/

#include "controller_library.hpp"
#include "Model.hpp"
#include "strutil.hpp"


namespace jspace {
  
  
  Status FloatController::
  setGoal(Vector const & goal)
  {
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  getGoal(Vector & goal) const
  {
    goal.resize(0);
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  getActual(Vector & actual) const
  {
    actual.resize(0);
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  setGains(Vector const & kp, Vector const & kd)
  {
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  getGains(Vector & kp, Vector & kd) const
  {
    kp.resize(0);
    kd.resize(0);
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  latch(Model const & model)
  {
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  computeCommand(Model const & model, Vector & tau)
  {
    Vector gg;
    model.getGravity(gg);
    tau.resize(gg.rows());
    memcpy(&tau[0], gg.data(), gg.rows() * sizeof(double));
    Status ok;
    return ok;
  }
  
  
  GoalControllerBase::
  GoalControllerBase(int compensation_flags,
		     Vector const & default_kp,
		     Vector const & default_kd)
    : compensation_flags_(compensation_flags),
      default_kp_(default_kp),
      default_kd_(default_kd)
  {
  }
  
  
  Status GoalControllerBase::
  init(Model const & model)
  {
    Status status;
    ssize_t const ndof(model.getNDOF());
    
    if (model.getState().position_.size() != ndof) {
      status.ok = false;
      status.errstr =
	"inconsistent model: ndof = " + sfl::to_string(ndof)
	+ " but state.size() = " + sfl::to_string(model.getState().position_.size());
      return status;
    }

    // If this is the first time we got called, then initialize gains.
    if (ndof != goal_.size()) {
      if ((ndof != default_kp_.size()) || (ndof != default_kd_.size())) {
	status.ok = false;
	status.errstr =
	  "inconsistent default gains: ndof = " + sfl::to_string(ndof)
	  + " but default_kp_.size() = " + sfl::to_string(default_kp_.size())
	  + " and default_kd_.size() = " + sfl::to_string(default_kd_.size());
	return status;
      }
      kp_ = default_kp_;
      kd_ = default_kd_;
    }
    
    // Set goal to current position.
    goal_ = model.getState().position_;
    
    return status;
  }
  
  
  Status GoalControllerBase::
  setGoal(Vector const & goal)
  {
    Status status;
    if (goal.size() != goal_.size()) {
      status.ok = false;
      status.errstr =
	"goal size mismatch: expected " + sfl::to_string(goal_.size())
	+ " but got " + sfl::to_string(goal.size());
      return status;
    }
    goal_ = goal;
    return status;
  }
  
  
  Status GoalControllerBase::
  getGoal(Vector & goal) const
  {
    goal = goal_;
    Status ok;
    return ok;
  }
  
  
  Status GoalControllerBase::
  setGains(Vector const & kp, Vector const & kd)
  {
    Status status;
    if ((kp.size() != kp_.size()) || (kd.size() != kd_.size())) {
      status.ok = false;
      status.errstr = "gain size mismatch (maybe not initialized?)";
      return status;
    }
    kp_ = kp;
    kd_ = kd;
    return status;
  }
  
  
  Status GoalControllerBase::
  getGains(Vector & kp, Vector & kd) const
  {
    kp = kp_;
    kd = kd_;
    Status ok;
    return ok;
  }
  
  
  JointGoalController::
  JointGoalController(int compensation_flags,
		      Vector const & default_kp,
		      Vector const & default_kd)
    : GoalControllerBase(compensation_flags, default_kp, default_kd)
  {
  }
  
  
  Status JointGoalController::
  getActual(Vector & actual) const
  {
    actual = actual_;
    Status ok;
    return ok;
  }
  
  
  Status JointGoalController::
  latch(Model const & model)
  {
    goal_ = model.getState().position_;
    Status ok;
    return ok;
  }
  
  
  Status JointGoalController::
  computeCommand(Model const & model, Vector & tau)
  {
    ssize_t const ndof(model.getNDOF());
    Status status;
    if (ndof != goal_.size()) {
      status.ok = false;
      status.errstr = "ndof mismatch";
      return status;
    }
    
    State const & state(model.getState());
    actual_ = state.position_;

    Vector etau(ndof);
    for (ssize_t ii(0); ii < ndof; ++ii) {
      etau[ii] = - kp_[ii] * (actual_[ii] - goal_[ii]) - kd_[ii] * state.velocity_[ii];
    }
    
    if (compensation_flags_ & COMP_MASS_INERTIA) {
      Matrix AA;
      if ( ! model.getMassInertia(AA)) {
	status.ok = false;
	status.errstr = "model.getMassInertia() failed";
	return status;
      }
      etau = AA * etau;
    }
    
    if (compensation_flags_ & COMP_CORIOLIS) {
      Vector BB;
      if ( ! model.getCoriolisCentrifugal(BB)) {
	status.ok = false;
	status.errstr = "model.getCoriolisCentrifugal() failed";
	return status;
      }
      etau += BB;
    }
    
    if (compensation_flags_ & COMP_GRAVITY) {
      Vector GG;
      if ( ! model.getGravity(GG)) {
	status.ok = false;
	status.errstr = "model.getGravity() failed";
	return status;
      }
      etau += GG;
    }
    
    tau.resize(etau.rows());
    memcpy(&tau[0], etau.data(), etau.rows() * sizeof(double));
    
    return status;
  }
  
}
