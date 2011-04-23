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
   \file util.hpp
   \author Roland Philippsen
*/

#include <jspace/wrap_eigen.hpp>
#include <stdexcept>
#include <vector>

namespace jspace {
  namespace test {
    
    double smart_delta(double have, double want, double epsilon = 1e-6);
    
    bool check_matrix(char const * name,
		      jspace::Matrix const & want,
		      jspace::Matrix const & have,
		      double precision,
		      std::ostringstream & msg);
    
    bool check_vector(char const * name,
		      jspace::Vector const & want,
		      jspace::Vector const & have,
		      double precision,
		      std::ostream & msg);
    
    bool check_vector(char const * name,
		      std::vector<double> const & want,
		      std::vector<double> const & have,
		      double precision,
		      std::ostream & msg);
    
    std::string create_tmpfile(char const * fname_template, char const * contents) throw(std::runtime_error);
    
  }
}
