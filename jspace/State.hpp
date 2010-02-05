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
   \file jspace/State.hpp
   \author Roland Philippsen
*/

#include <saimatrix/SAIVector.h>
#include <sys/time.h>

namespace jspace {
  
  class State
  {
  public:
    State();
    State(int npos, int nvel);
    
    void init(int npos, int nvel);
    State & operator = (State const & rhs);
    
    timeval acquisition_time_;
    SAIVector joint_angles_;
    SAIVector joint_velocities_;
  };
  
}
