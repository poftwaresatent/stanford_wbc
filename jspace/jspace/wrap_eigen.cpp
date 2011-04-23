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
   \file jspace/wrap_eigen.cpp
   \author Roland Philippsen
*/

#include "wrap_eigen.hpp"
#include <stdio.h>

using namespace std;

namespace jspace {
  
  bool compare(jspace::Matrix const & lhs, jspace::Matrix const & rhs, double precision)
  {
    if ( &lhs == &rhs ) {
      return true;
    }
    if ( lhs.rows() != rhs.rows() ) {
      return false;
    }
    if ( lhs.cols() != rhs.cols() ) {
      return false;
    }
    for (int ii(0); ii < lhs.rows(); ++ii) {
      for (int jj(0); jj < lhs.cols(); ++jj) {
	if (fabs(lhs.coeff(ii, jj) - rhs.coeff(ii, jj)) > precision) {
	  return false;
	}
      }
    }
    return true;
  }
    
    
  bool compare(jspace::Quaternion const & lhs, jspace::Quaternion const & rhs, double precision)
  {
    return compare(lhs.coeffs(), rhs.coeffs(), precision);
  }
  
  
  std::string pretty_string(jspace::Vector const & vv)
  {
    ostringstream os;
    pretty_print(vv, os, "", "", true);
    return os.str();
  }
  
    
  std::string pretty_string(jspace::Quaternion const & qq)
  {
    ostringstream os;
    pretty_print(qq, os, "", "", true);
    return os.str();
  }
    
    
  std::string pretty_string(jspace::Matrix const & mm, std::string const & prefix)
  {
    ostringstream os;
    pretty_print(mm, os, "", prefix);
    return os.str();
  }
    
    
  void pretty_print(jspace::Vector const & vv, std::ostream & os,
		    std::string const & title, std::string const & prefix,
		    bool nonl)
  {
    pretty_print((jspace::Matrix const &) vv, os, title, prefix, true, nonl);
  }
    
    
  void pretty_print(jspace::Quaternion const & qq, std::ostream & os,
		    std::string const & title, std::string const & prefix,
		    bool nonl)
  {
    pretty_print(qq.coeffs(), os, title, prefix, true, nonl);
  }
    
    
  std::string pretty_string(double vv)
  {
    static int const buflen(32);
    static char buf[buflen];
    memset(buf, 0, sizeof(buf));
#ifndef WIN32
    if (isinf(vv)) {
      snprintf(buf, buflen-1, " inf    ");
    }
    else if (isnan(vv)) {
      snprintf(buf, buflen-1, " nan    ");
    }
    else if (fabs(fmod(vv, 1)) < 1e-6) {
      snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(vv)));
    }
    else {
      snprintf(buf, buflen-1, "% 6.4f  ", vv);
    }
#else // WIN32
    sprintf_s(buf, buflen-1, "% 6.4f  ", vv);
#endif // WIN32
    string str(buf);
    return str;
  }
  
  
  void pretty_print(jspace::Matrix const & mm, std::ostream & os,
		    std::string const & title, std::string const & prefix,
		    bool vecmode, bool nonl)
  {
    char const * nlornot("\n");
    if (nonl) {
      nlornot = "";
    }
    if ( ! title.empty()) {
      os << title << nlornot;
    }
    if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
      os << prefix << " (empty)" << nlornot;
    }
    else {
      // if (mm.cols() == 1) {
      //   vecmode = true;
      // }
	
      if (vecmode) {
	if ( ! prefix.empty())
	  os << prefix;
	for (int ir(0); ir < mm.rows(); ++ir) {
	  os << pretty_string(mm.coeff(ir, 0));
	}
	os << nlornot;
	
      }
      else {

	for (int ir(0); ir < mm.rows(); ++ir) {
	  if ( ! prefix.empty())
	    os << prefix;
	  for (int ic(0); ic < mm.cols(); ++ic) {
	    os << pretty_string(mm.coeff(ir, ic));
	  }
	  os << nlornot;
	}
	  
      }
    }
  }
  
  
  void convert(jspace::Vector const & from, std::vector<double> & to)
  {
    to.resize(from.size());
    Vector::Map(&to[0], to.size()) = from;
  }
  
  
  void convert(std::vector<double> const & from, jspace::Vector & to)
  {
    to = Vector::Map(&from[0], from.size());
  }
  
  
  void convert(double const * from, size_t length, jspace::Vector & to)
  {
    to = Vector::Map(from, length);
  }
  
}
