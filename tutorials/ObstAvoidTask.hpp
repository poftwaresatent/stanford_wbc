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
   \file ObstAvoidTask.hpp
   \author Roland Philippsen
*/

#ifndef PWS_OBSTAVOID_TASK_HPP
#define PWS_OBSTAVOID_TASK_HPP

#include <opspace/task_library.hpp>

namespace pws {
  
  using namespace opspace;
  
  
  class ObstAvoidTask
    : public opspace::PDTask
  {
  public:
    explicit ObstAvoidTask(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual Status check(std::string const * param, std::string const & value) const;
    
    inline void quickSetup(double kp, double kd, double maxvel,
			   double dsafe, jspace::Vector const & local_control_point,
			   std::string const & link_name)
    {
      PDTask::quickSetup(Vector::Ones(1) * kp,
			 Vector::Ones(1) * kd,
			 Vector::Ones(1) * maxvel);
      dsafe_ = dsafe;
      link_name_ = link_name;
      local_control_point_ = local_control_point;
    }
    
  protected:
    std::string link_name_;
    Vector global_obstacle_;
    Vector global_delta_;
    Vector global_unit_;
    Vector global_control_point_;
    Vector local_control_point_;
    Matrix jac_x_;
    double dsafe_;
    double activation_;
    
    mutable taoDNode const * node_;
    
    taoDNode const * updateActual(Model const & model);
  };

}

#endif // PWS_OBSTAVOID_TASK_HPP
