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
   \file ShuffleVector.hpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2009 Roland Philippsen, released under a BSD license.
*/

#ifndef WBC_SHUFFLE_VECTOR_HPP
#define WBC_SHUFFLE_VECTOR_HPP

#include <sys/types.h>
#include <vector>
#include <iosfwd>

class SAIVector;

namespace wbc {

  class ShuffleVector
  {
  public:
    explicit ShuffleVector(double empty_value);
  
    void SetShuffle(size_t from_index, size_t to_index);
  
    bool ShuffleDirect(double const * in, size_t in_len, double * out, size_t out_len) const;
    bool ShuffleInverse(double const * in, size_t in_len, double * out, size_t out_len) const;
  
    bool ShuffleDirect(SAIVector const & in, SAIVector & out) const;
    bool ShuffleInverse(SAIVector const & in, SAIVector & out) const;
  
    typedef std::vector<ssize_t> shuffle_t; // -1 means 'use empty value'
  
  protected:
    double m_empty_value;
    shuffle_t m_direct;
    shuffle_t m_inverse;
  };

}

#endif // WBC_SHUFFLE_VECTOR_HPP
