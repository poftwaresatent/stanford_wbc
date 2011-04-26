/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * Author: Roland Philippsen
 *         http://cs.stanford.edu/group/manips/
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

#include <opspace/TypeIOTGCursor.hpp>

namespace opspace {
  
  
  TypeIOTGCursor::
  TypeIOTGCursor(size_t ndof, double dt_seconds)
    : ndof_(ndof),
      dt_seconds_(dt_seconds),
      otg_(ndof, dt_seconds)
  {
    pos_clean_ = Vector::Zero(ndof);
    vel_clean_ = Vector::Zero(ndof);
    pos_dirty_ = Vector::Zero(ndof);
    vel_dirty_ = Vector::Zero(ndof);
    selection_.resize(ndof);
    for (size_t ii(0); ii < ndof; ++ii) {
      selection_[ii] = true;
    }
  }
  
  
  int TypeIOTGCursor::
  next(Vector const & maxvel,
       Vector const & maxacc,
       Vector const & goal)
  {
    int const result(otg_.GetNextMotionState_Position(pos_clean_.data(),
						      vel_clean_.data(),
						      maxvel.data(),
						      maxacc.data(),
						      goal.data(),
						      selection_.data(),
						      pos_dirty_.data(),
						      vel_dirty_.data()));
    if (0 <= result) {
      pos_clean_ = pos_dirty_;
      vel_clean_ = vel_dirty_;
    }
    return result;
  }
  
  
  int TypeIOTGCursor::
  next(double maxvel,
       double maxacc,
       double goal)
  {
    if (ndof_ != 1) {
      return -1000;
    }
    int const result(otg_.GetNextMotionState_Position(pos_clean_.data(),
						      vel_clean_.data(),
						      &maxvel,
						      &maxacc,
						      &goal,
						      selection_.data(),
						      pos_dirty_.data(),
						      vel_dirty_.data()));
    if (0 <= result) {
      pos_clean_ = pos_dirty_;
      vel_clean_ = vel_dirty_;
    }
    return result;
  }
  
  
  char const * otg_errstr(int otg_error_code)
  {
    switch (otg_error_code) {
    case TypeIOTG::OTG_ERROR:
      return "OTG_ERROR (general error)";
    case TypeIOTG::OTG_MAX_VELOCITY_ERROR:
      return "OTG_MAX_VELOCITY_ERROR (maxvel too small)";
    case TypeIOTG::OTG_MAX_ACCELERATION_ERROR:
      return "OTG_MAX_ACCELERATION_ERROR (maxacc too small)";
    case TypeIOTG::OTG_WORKING:
      return "OTG_WORKING (everything is fine)";
    case TypeIOTG::OTG_FINAL_STATE_REACHED:
      return "OTG_FINAL_STATE_REACHED (all is fine and we're at the goal)";
    }
    return "invalid or unrecognized otg_error_code";
  }
  
}
