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
#include "vector_util.hpp"
#include <string.h>


namespace jspace {
  
  
  State::
  State()
  {
    init(0, 0, 0);
  }
  
  
  State::
  State(State const & orig)
  {
    *this = orig;
  }
  
  
  State::
  State(size_t npos, size_t nvel, size_t nforce)
  {
    init(npos, nvel, nforce);
  }
  
  
  void State::
  init(size_t npos, size_t nvel, size_t nforce)
  {
    time_sec_ = 0;
    time_usec_ = 0;
    position_.resize(npos);
    velocity_.resize(nvel);
    force_.resize(nforce);
    memset(&position_[0], 0, npos * sizeof(double));
    memset(&velocity_[0], 0, nvel * sizeof(double));
    memset(&force_[0], 0, nforce * sizeof(double));
  }
  
  
  State & State::
  operator = (State const & rhs)
  {
    if (&rhs == this) {
      return *this;
    }
    time_sec_ = rhs.time_sec_;
    time_usec_ = rhs.time_usec_;
    position_ = rhs.position_;
    velocity_ = rhs.velocity_;
    force_ = rhs.force_;
    return *this;
  }
  
  
  bool State::
  equal(State const & rhs, int flags, double precision) const
  {
    if (&rhs == this) {
      return true;
    }
    if (flags & COMPARE_ACQUISITION_TIME) {
      if (time_sec_ != rhs.time_sec_) {
	return false;
      }
      if (time_usec_ != rhs.time_usec_) {
	return false;
      }
    }
    if (flags & COMPARE_POSITION) {
      if ( ! compare(position_, rhs.position_, precision)) {
	return false;
      }
    }
    if (flags & COMPARE_VELOCITY) {
      if ( ! compare(velocity_, rhs.velocity_, precision)) {
	return false;
      }
    }
    if (flags & COMPARE_FORCE) {
      if ( ! compare(force_, rhs.force_, precision)) {
	return false;
      }
    }
    return true;
  }
  
}
