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

#ifndef OPSPACE_TYPE_I_OTG_CURSOR_HPP
#define OPSPACE_TYPE_I_OTG_CURSOR_HPP

#include <jspace/wrap_eigen.hpp>
#include <reflexxes_otg/TypeIOTG.h>

namespace opspace {
  
  using jspace::Vector;

  char const * otg_errstr(int otg_error_code);
  
  
  /**
     Utility for using reflexxes_otg::TypeIOTG. This class wraps a
     acceleration-bounded trajectory object from the reflexxes_otg
     library and makes it easier to use, e.g. inside
     opspace::TrajectoryTask.
     
     The idea is that you simply initialize it by setting the starting
     position() and velocity(), and then repeatedly call next() to
     advance to the next desired position and velocity.
  */
  class TypeIOTGCursor
  {
  public:
    size_t const ndof_;
    double const dt_seconds_;
    
    TypeIOTGCursor(size_t ndof,
		   double dt_seconds);
    
    /**
       Compute the next desired position() and velocity(). You specify
       the maximum velocity and acceleration right here, as well as
       the goal. This allows you to change all these parameters on the
       fly without needing to mess with the cursor's internal state.
       
       If you want to change the starting state of the trajectory,
       simply assign to position() and velocity(). You typically do
       that only at (re-)initialization time.
       
       \return reflexxes_otg::TypeIOTGResult which is easy to
       interpret: 0 on success, 1 if we were already at the goal
       anyway, and negative values on error.
    */
    int next(Vector const & maxvel,
	     Vector const & maxacc,
	     Vector const & goal);

    /**
       In case of one-dimensional cursor, this is easier to
       use. Returns -1000 in case the dimension is not 1.
    */
    int next(double maxvel,
	     double maxacc,
	     double goal);
    
    inline Vector & position()             { return pos_clean_; }
    inline Vector const & position() const { return pos_clean_; }
    inline Vector & velocity()             { return vel_clean_; }
    inline Vector const & velocity() const { return vel_clean_; }
    
  protected:
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> boolvec_t;
    
    TypeIOTG otg_;
    boolvec_t selection_;
    Vector pos_clean_;
    Vector vel_clean_;
    Vector pos_dirty_;
    Vector vel_dirty_;
  };
  
}

#endif // OPSPACE_TYPE_I_OTG_CURSOR_HPP
