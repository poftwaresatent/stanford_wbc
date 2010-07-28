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
   \file jspace/controller_library.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_CONTROLLER_LIBRARY_HPP
#define JSPACE_CONTROLLER_LIBRARY_HPP

#include <jspace/Controller.hpp>
#include <string>


namespace jspace {
  
  
  typedef enum {
    COMP_NONE         = 0x00,
    COMP_GRAVITY      = 0x01,
    COMP_CORIOLIS     = 0x02,
    COMP_MASS_INERTIA = 0x04
  } compensation_flags_t;
  
  
  class FloatController
    : public Controller
  {
  public:
    virtual Status setGoal(std::vector<double> const & goal);
    virtual Status getGoal(std::vector<double> & goal) const;
    virtual Status getActual(std::vector<double> & actual) const;
    
    virtual Status setGains(std::vector<double> const & kp, std::vector<double> const & kd);
    virtual Status getGains(std::vector<double> & kp, std::vector<double> & kd) const;
    
    virtual Status latch(Model const & model);
    virtual Status computeCommand(Model const & model, std::vector<double> & tau);
  };
  
  
  class GoalControllerBase
    : public Controller
  {
  public:
    GoalControllerBase(int compensation_flags,
		       std::vector<double> const & default_kp,
		       std::vector<double> const & default_kd);
    
    virtual Status init(Model const & model);
    
    virtual Status setGoal(std::vector<double> const & goal);
    virtual Status getGoal(std::vector<double> & goal) const;
    
    virtual Status setGains(std::vector<double> const & kp, std::vector<double> const & kd);
    virtual Status getGains(std::vector<double> & kp, std::vector<double> & kd) const;
    
  protected:
    int compensation_flags_;
    std::vector<double> default_kp_;
    std::vector<double> default_kd_;
    std::vector<double> goal_;
    std::vector<double> kp_;
    std::vector<double> kd_;
  };
  
  
  class JointGoalController
    : public GoalControllerBase
  {
  public:
    JointGoalController(int compensation_flags,
		       std::vector<double> const & default_kp,
		       std::vector<double> const & default_kd);
    
    virtual Status getActual(std::vector<double> & actual) const;
    virtual Status latch(Model const & model);
    virtual Status computeCommand(Model const & model, std::vector<double> & tau);
    
  protected:
    std::vector<double> actual_;
  };
  
}

#endif // JSPACE_CONTROLLER_LIBRARY_HPP
