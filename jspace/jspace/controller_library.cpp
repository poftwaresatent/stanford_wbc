/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
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
   \file jspace/controller_library.cpp
   \author Roland Philippsen
*/

#include "controller_library.hpp"
#include "Model.hpp"
#include <wbcnet/strutil.hpp>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>


namespace jspace {
  
  
  Status FloatController::
  setGoal(std::vector<double> const & goal)
  {
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  getGoal(std::vector<double> & goal) const
  {
    goal.clear();
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  getActual(std::vector<double> & actual) const
  {
    actual.clear();
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  setGains(std::vector<double> const & kp, std::vector<double> const & kd)
  {
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  getGains(std::vector<double> & kp, std::vector<double> & kd) const
  {
    kp.clear();
    kd.clear();
    Status ok;
    return ok;
  }
  
  
  Status FloatController::
  computeCommand(Model const & model, std::vector<double> & tau)
  {
    SAIVector gg;
    model.getGravity(gg);
    gg.getValues(tau);
    Status ok;
    return ok;
  }
  
  
  GoalControllerBase::
  GoalControllerBase(int compensation_flags,
		     double default_kp,
		     double default_kd)
    : compensation_flags_(compensation_flags),
      default_kp_(default_kp),
      default_kd_(default_kd)
  {
  }
  
  
  Status GoalControllerBase::
  init(Model const & model)
  {
    Status status;
    size_t const ndof(model.getNDOF());
    
    if (model.getState().position_.size() != ndof) {
      status.ok = false;
      status.errstr =
	"inconsistent model: ndof = " + sfl::to_string(ndof)
	+ " but state.size() = " + sfl::to_string(model.getState().position_.size());
      return status;
    }
    
    // If this is the first time we got called, initialize to default.
    if (ndof != goal_.size()) {
      goal_.resize(ndof);
      kp_.resize(ndof);
      kd_.resize(ndof);
      for (size_t ii(0); ii < ndof; ++ii) {
	kp_[ii] = default_kp_;
	kd_[ii] = default_kd_;
      }
    }
    
    // Set goal to current position.
    goal_ = model.getState().position_;
    
    return status;
  }
  
  
  Status GoalControllerBase::
  setGoal(std::vector<double> const & goal)
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
  getGoal(std::vector<double> & goal) const
  {
    goal = goal_;
    Status ok;
    return ok;
  }
  
  
  Status GoalControllerBase::
  setGains(std::vector<double> const & kp, std::vector<double> const & kd)
  {
    Status status;
    if ((kp.size() != kp_.size()) || (kd.size() != kd_.size())) {
      status.ok = false;
      status.errstr = "gain size mismatch";
      return status;
    }
    kp_ = kp;
    kd_ = kd;
    return status;
  }
  
  
  Status GoalControllerBase::
  getGains(std::vector<double> & kp, std::vector<double> & kd) const
  {
    kp = kp_;
    kd = kd_;
    Status ok;
    return ok;
  }
  
  
  JointGoalController::
  JointGoalController(int compensation_flags,
		      double default_kp,
		      double default_kd)
    : GoalControllerBase(compensation_flags, default_kp, default_kd)
  {
  }
  
  
  Status JointGoalController::
  getActual(std::vector<double> & actual) const
  {
    actual = actual_;
    Status ok;
    return ok;
  }
  
  
  Status JointGoalController::
  computeCommand(Model const & model, std::vector<double> & tau)
  {
    size_t const ndof(model.getNDOF());
    Status status;
    if (ndof != goal_.size()) {
      status.ok = false;
      status.errstr = "ndof mismatch";
      return status;
    }
    
    State const & state(model.getState());
    actual_ = state.position_;

    //DEBUG//     cerr << "JointGoalController\n";
    
    SAIVector sai_tau(ndof);
    for (size_t ii(0); ii < ndof; ++ii) {
      sai_tau[ii] = - kp_[ii] * (actual_[ii] - goal_[ii]) - kd_[ii] * state.velocity_[ii];
    }
    
    //DEBUG//     sai_tau.prettyPrint(cerr, "  raw tau", "    ");
    
    if (compensation_flags_ & COMP_MASS_INERTIA) {
      SAIMatrix AA;
      if ( ! model.getMassInertia(AA)) {
	status.ok = false;
	status.errstr = "model.getMassInertia() failed";
	return status;
      }
      sai_tau = AA * sai_tau;
      
      //DEBUG//       AA.prettyPrint(cerr, "  MassInertia", "    ");
      //DEBUG//       sai_tau.prettyPrint(cerr, "  after COMP_MASS_INERTIA", "    ");

    }
    
    if (compensation_flags_ & COMP_CORIOLIS) {
      SAIVector BB;
      if ( ! model.getCoriolisCentrifugal(BB)) {
	status.ok = false;
	status.errstr = "model.getCoriolisCentrifugal() failed";
	return status;
      }
      sai_tau += BB;
      
      //DEBUG//       sai_tau.prettyPrint(cerr, "  after COMP_CORIOLIS", "    ");

    }
    
    if (compensation_flags_ & COMP_GRAVITY) {
      SAIVector GG;
      if ( ! model.getGravity(GG)) {
	status.ok = false;
	status.errstr = "model.getGravity() failed";
	return status;
      }
      sai_tau += GG;
      
      //DEBUG//       sai_tau.prettyPrint(cerr, "  after COMP_GRAVITY", "    ");
      
    }
    
    sai_tau.getValues(tau);
    return status;
  }
  
}
