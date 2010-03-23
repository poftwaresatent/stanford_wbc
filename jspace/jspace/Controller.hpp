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
   \file jspace/Controller.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_CONTROLLER_HPP
#define JSPACE_CONTROLLER_HPP

#include <jspace/Status.hpp>
#include <vector>


namespace jspace {
  
  class Model;
  
  
  class Controller
  {
  public:
    virtual ~Controller() {}
    
    /** Default init just returns ok. */
    virtual Status init(Model const & model) { Status ok; return ok; }
    
    virtual Status setGoal(std::vector<double> const & goal) = 0;
    virtual Status getGoal(std::vector<double> & goal) const = 0;
    virtual Status getActual(std::vector<double> & actual) const = 0;
    
    virtual Status setGains(std::vector<double> const & kp, std::vector<double> const & kd) = 0;
    virtual Status getGains(std::vector<double> & kp, std::vector<double> & kd) const = 0;
    
    virtual Status computeCommand(Model const & model, std::vector<double> & tau) = 0;
  };
  
}

#endif // JSPACE_CONTROLLER_HPP
