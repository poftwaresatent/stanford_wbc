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
   \file jspace/State.cpp
   \author Roland Philippsen
*/

#include "State.hpp"

namespace jspace {
  
  
  State::
  State()
  {
    init(0, 0);
  }
  
  
  State::
  State(State const & orig)
  {
    init(0, 0);
    *this = orig;
  }
  
  
  State::
  State(int npos, int nvel)
  {
    init(npos, nvel);
  }
  
  
  void State::
  init(int npos, int nvel)
  {
    acquisition_time_.tv_sec = 0;
    acquisition_time_.tv_usec = 0;
    joint_angles_.setSize(npos, true);
    joint_velocities_.setSize(npos, true);
  }
  
  
  State & State::
  operator = (State const & rhs)
  {
    if (&rhs == this) {
      return *this;
    }
    acquisition_time_.tv_sec = rhs.acquisition_time_.tv_sec;
    acquisition_time_.tv_usec = rhs.acquisition_time_.tv_usec;
    joint_angles_ = rhs.joint_angles_;
    joint_velocities_ = rhs.joint_velocities_;
    return *this;
  }
  
  
  bool State::
  equal(State const & rhs, int flags, double precision) const
  {
    if (&rhs == this) {
      return true;
    }
    if (flags & COMPARE_ACQUISITION_TIME) {
      if (acquisition_time_.tv_sec != rhs.acquisition_time_.tv_sec) {
	return false;
      }
      if (acquisition_time_.tv_usec != rhs.acquisition_time_.tv_usec) {
	return false;
      }
    }
    if (flags & COMPARE_JOINT_ANGLES) {
      if ( ! joint_angles_.equal(rhs.joint_angles_, precision)) {
	return false;
      }
    }
    if (flags & COMPARE_JOINT_VELOCITIES) {
      if ( ! joint_velocities_.equal(rhs.joint_velocities_, precision)) {
	return false;
      }
    }
    return true;
  }
  
}
