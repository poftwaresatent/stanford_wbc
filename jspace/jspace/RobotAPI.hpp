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

#ifndef JSPACE_ROBOT_API_HPP
#define JSPACE_ROBOT_API_HPP

#include <jspace/State.hpp>
#include <jspace/Status.hpp>


namespace jspace {
  
  
  /**
     Abstract API for talking to a robot. We use generalized
     coordinates (position and velocity) and generalized forces (joint
     torques) as basic abstraction.
  */
  class RobotAPI
  {
  public:
    virtual ~RobotAPI() {}
    
    virtual Status readState(State & state) = 0;
    
    virtual Status writeCommand(Vector const & command) = 0;
    
    virtual void shutdown() = 0;
  };
  
}

#endif // JSPACE_ROBOT_API_HPP
