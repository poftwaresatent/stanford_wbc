/*
 * Copyright (C) 2013 Roland Philippsen. All rights reserved.
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
   \file ObstAvoidTask.cpp
   \author Roland Philippsen
*/

#include "ObstAvoidTask.hpp"

namespace pws {
  
  
  ObstAvoidTask::
  ObstAvoidTask(std::string const & name)
    : PDTask(name, PDTask::SATURATION_NORM),
      link_name_(""),
      local_control_point_(Vector::Zero(3)),
      dsafe_(0.5),
      activation_(-1),
      node_(0)
  {
    declareParameter("link", &link_name_, PARAMETER_FLAG_NOLOG);
    declareParameter("activation", &activation_, PARAMETER_FLAG_READONLY);
    declareParameter("dsafe", &dsafe_, PARAMETER_FLAG_NOLOG);
    declareParameter("global_delta", &global_delta_, PARAMETER_FLAG_READONLY);
    declareParameter("global_unit", &global_unit_, PARAMETER_FLAG_READONLY);
    declareParameter("global_obstacle", &global_obstacle_);
    declareParameter("local_control_point", &local_control_point_);
    declareParameter("global_control_point", &global_control_point_, PARAMETER_FLAG_READONLY);
  }
  
  
  Status ObstAvoidTask::
  init(Model const & model)
  {
    if (link_name_.empty()) {
      return Status(false, "no link");
    }
    if ((1 != kp_.rows()) || (1 != kd_.rows()) || (1 != maxvel_.rows()))  {
      return Status(false, "kp, kd, and maxvel must be 1-dimensional");
    }
    node_ = updateActual(model);
    if ( ! node_) {
      return Status(false, "invalid link or unsupported joint type");
    }
    return initPDTask(actual_);
  }
  
  
  Status ObstAvoidTask::
  update(Model const & model)
  {
    node_ = updateActual(model);
    if ( ! node_) {
      return Status(false, "invalid link or unsupported joint type");
    }
    
    if (0 > activation_) {
      jacobian_ = Matrix::Zero(0, 0);
      command_ = Vector::Zero(1);
      return Status();
    }
    
    Status st(computePDCommand(actual_,
			       jacobian_ * model.getState().velocity_,
			       command_));
    if (st) {
      command_ *= activation_;
    }
    else {
      command_ = Vector::Zero(1);
    }
    return st;
  }
  
  
  Status ObstAvoidTask::
  check(std::string const * param, std::string const & value) const
  {
    if (param == &link_name_) {
      node_ = 0; // lazy re-init... would be nice to detect errors here though, but need model
    }
    Status ok;
    return ok;
  }
  
  
  taoDNode const * ObstAvoidTask::
  updateActual(Model const & model)
  {
    if ( ! node_) {
      node_ = model.getNodeByName(link_name_);
    }
    if ( ! node_) {
      return 0;
    }
    
    // jspace::pretty_print(global_obstacle_, std::cout,
    // 			 "obstavoid update global_obstacle_", "  ");

    jspace::Transform ee_transform;
    model.computeGlobalFrame(node_,
			     local_control_point_[0],
			     local_control_point_[1],
			     local_control_point_[2],
			     ee_transform);
    global_control_point_ = ee_transform.translation();

    // jspace::pretty_print(global_control_point_, std::cout,
    // 			 "obstavoid update global_control_point_", "  ");
    
    global_delta_ = global_obstacle_ - global_control_point_;

    // jspace::pretty_print(global_delta_, std::cout,
    // 			 "obstavoid update global_delta_", "  ");

    double const dd(global_delta_.norm());
    global_unit_ = global_delta_ / dd;

    // jspace::pretty_print(global_unit_, std::cout,
    // 			 "obstavoid update global_unit_", "  ");
    
    jspace::Matrix jfull;
    if ( ! model.computeJacobian(node_,
				 global_control_point_[0],
				 global_control_point_[1],
				 global_control_point_[2],
				 jfull)) {
      return 0;
    }
    jac_x_ = jfull.block(0, 0, 3, jfull.cols());

    // jspace::pretty_print(jac_x_, std::cout,
    // 			 "obstavoid update jac_x_", "  ");
    
    jacobian_ = - global_unit_.transpose() * jac_x_;

    // jspace::pretty_print(jacobian_, std::cout,
    // 			 "obstavoid update jacobian_", "  ");
    
    if ((0 <= activation_) && (dd > 1.5 * dsafe_)) {
      activation_ = -1;
    }
    if ((0 <= activation_) || (dd <= dsafe_)) {
      if (dd > dsafe_) {
	activation_ = 0.1;
      }
      else if (dd <= 0.5 * dsafe_) {
	activation_ = 1.0;
      }
      else {
	activation_ = 1.9 - dd * 1.8 / dsafe_;
      }
    }
    
    actual_ = Vector::Ones(1) * dd;
    goalpos_ = Vector::Ones(1) * 3 * dsafe_; // XXXX to do: never changes
    goalvel_ = Vector::Zero(1);	// XXXX to do: never changes
    
    return node_;
  }
}
