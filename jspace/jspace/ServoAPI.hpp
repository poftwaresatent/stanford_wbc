/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2008-2010 Stanford University
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

#ifndef JSPACE_SERVO_API_HPP
#define JSPACE_SERVO_API_HPP

#include <jspace/State.hpp>
#include <jspace/Status.hpp>


namespace jspace {
  
  
  class ServoInfo
  {
  public:
    std::vector<std::string> controller_name;
    std::vector<std::string> dof_name;
    std::vector<double> limit_lower;
    std::vector<double> limit_upper;
  };
  
  
  class ServoState
  {
  public:
    std::string active_controller;
    std::vector<double> goal;
    std::vector<double> actual;
    std::vector<double> kp;
    std::vector<double> kd;
  };
  
  
  /**
     Abstract API for talking to a servo.
  */
  class ServoAPI
  {
  public:
    virtual ~ServoAPI() {}
    
    virtual Status getInfo(ServoInfo & info) const = 0;
    
    virtual Status getState(ServoState & state) const = 0;
    
    virtual Status selectController(std::string const & name) = 0;
    
    virtual Status setGoal(std::vector<double> const & goal) = 0;
    
    virtual Status setGains(std::vector<double> const & kp, std::vector<double> const & kd) = 0;
  };
  
}

#endif // JSPACE_SERVO_API_HPP
