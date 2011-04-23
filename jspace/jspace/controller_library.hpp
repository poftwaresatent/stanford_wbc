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
    virtual Status setGoal(Vector const & goal);
    virtual Status getGoal(Vector & goal) const;
    virtual Status getActual(Vector & actual) const;
    
    virtual Status setGains(Vector const & kp, Vector const & kd);
    virtual Status getGains(Vector & kp, Vector & kd) const;
    
    virtual Status latch(Model const & model);
    virtual Status computeCommand(Model const & model, Vector & tau);
  };
  
  
  class GoalControllerBase
    : public Controller
  {
  public:
    GoalControllerBase(int compensation_flags,
		       Vector const & default_kp,
		       Vector const & default_kd);
    
    virtual Status init(Model const & model);
    
    virtual Status setGoal(Vector const & goal);
    virtual Status getGoal(Vector & goal) const;
    
    virtual Status setGains(Vector const & kp, Vector const & kd);
    virtual Status getGains(Vector & kp, Vector & kd) const;
    
  protected:
    int compensation_flags_;
    Vector default_kp_;
    Vector default_kd_;
    Vector goal_;
    Vector kp_;
    Vector kd_;
  };
  
  
  class JointGoalController
    : public GoalControllerBase
  {
  public:
    JointGoalController(int compensation_flags,
			Vector const & default_kp,
			Vector const & default_kd);
    
    virtual Status getActual(Vector & actual) const;
    virtual Status latch(Model const & model);
    virtual Status computeCommand(Model const & model, Vector & tau);
    
  protected:
    Vector actual_;
  };
  
}

#endif // JSPACE_CONTROLLER_LIBRARY_HPP
