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
    
    // should also work as-is for jspace::Vector
    bool equal(jspace::Matrix const & lhs, jspace::Matrix const & rhs, double precision);
    bool equal(jspace::Quaternion const & lhs, jspace::Quaternion const & rhs, double precision);
    
    std::string pretty_string(jspace::Vector const & vv);
    std::string pretty_string(jspace::Quaternion const & qq);
    std::string pretty_string(jspace::Matrix const & mm, std::string const & prefix);
    
    void pretty_print(jspace::Vector const & vv, std::ostream & os,
		      std::string const & title, std::string const & prefix, bool nonl = false);
    
    void pretty_print(jspace::Quaternion const & qq, std::ostream & os,
		      std::string const & title, std::string const & prefix, bool nonl = false);
    
    void pretty_print(jspace::Matrix const & mm, std::ostream & os,
		      std::string const & title, std::string const & prefix,
		      bool vecmode = false, bool nonl = false);
    
  }
}
