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
   \file jspace/vector_util.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_VECTOR_UTIL_HPP
#define JSPACE_VECTOR_UTIL_HPP

#include <vector>
#include <iosfwd>
#include <math.h>


namespace jspace {
  
  
  template<typename container_t>
  bool compare(container_t const & lhs, container_t const & rhs, typename container_t::value_type precision)
  {
    if (&lhs == &rhs) {
      return true;
    }
    if (lhs.size() != rhs.size()) {
      return false;
    }
    typename container_t::const_iterator il(lhs.begin());
    typename container_t::const_iterator ir(rhs.begin());
    typename container_t::const_iterator il_end(lhs.end());
    for (/**/; il != il_end; ++il, ++ir) {
      if (fabs(*il - *ir) > precision) {
	return false;
      }
    }
    return true;
  }
  
  
  template<typename container_t>
  void zero(container_t & vv)
  {
    std::fill(vv.begin(), vv.end(), 0);
  }
  
}

namespace std {
  
  ostream & operator << (ostream & os, vector<double> const & rhs);
  
}

#endif // JSPACE_VECTOR_UTIL_HPP
