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
    // Grrr, Eigen2 does not gracefully handle MatrixBase::Zero(0);
    if (0 == npos) {
      position_.resize(0);
    }
    else {
      position_ = Vector::Zero(npos);
    }
    if (0 == nvel) {
      velocity_.resize(0);
    }
    else {
      velocity_ = Vector::Zero(nvel);
    }
    if (0 == nforce) {
      force_.resize(0);
    }
    else {
      force_ = Vector::Zero(nforce);
    }
  }
  
  
  static void _pad(Vector & vv, size_t nn)
  {
    size_t const old(vv.size());
    if (old != nn) {
      vv.resize(nn);
      if (old < nn) {
	memset(&vv[old], 0, (nn - old) * sizeof(double));
      }
    }
  }
  
  
  void State::
  resizeAndPadWithZeros(size_t npos, size_t nvel, size_t nforce)
  {
    _pad(position_, npos);
    _pad(velocity_, nvel);
    _pad(force_, nforce);
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
