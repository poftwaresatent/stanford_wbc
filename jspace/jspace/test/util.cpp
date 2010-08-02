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
   \file util.cpp
   \author Roland Philippsen
*/

#include "util.hpp"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

using namespace std;

namespace jspace {
  namespace test {
    
    
    double smart_delta(double have, double want, double epsilon)
    {
      if (fabs(want) > epsilon) {
        return (have - want) / want;
      }
      return have - want;
    }


    bool check_matrix(char const * name,
		      jspace::Matrix const & want,
		      jspace::Matrix const & have,
		      double precision,
		      std::ostringstream & msg)
    {
      int const nrows(want.rows());
      if (nrows != have.rows()) {
	msg << "check_matrix(" << name << ") size mismatch: have " << have.rows()
	    << " rows but want " << nrows << "\n";
	return false;
      }
      int const ncolumns(want.cols());
      if (ncolumns != have.cols()) {
	msg << "check_matrix(" << name << ") size mismatch: have " << have.cols()
	    << " columns but want " << ncolumns << "\n";
	return false;
      }
  
      precision = fabs(precision);
      double maxdelta(0);
      jspace::Matrix delta(nrows, ncolumns);
      for (int ii(0); ii < nrows; ++ii) {
	for (int jj(0); jj < ncolumns; ++jj) {
	  delta.coeffRef(ii, jj) = fabs(smart_delta(have.coeff(ii, jj), want.coeff(ii, jj)));
	  if (delta.coeff(ii, jj) > precision) {
	    maxdelta = delta.coeff(ii, jj);
	  }
	}
      }
      double const halfmax(0.5 * maxdelta);
      double const tenprecision(10 * precision);
  
      if (maxdelta <= precision) {
	msg << "check_matrix(" << name << ") OK\n";
      }
      else {
	msg << "check_matrix(" << name << ") FAILED\n";
      }
      msg << "  precision = " << precision << "\n"
	  << "  maxdelta = " << maxdelta << "\n";
      pretty_print(delta, msg, "  delta", "    ");
      msg << "  error pattern\n";
      for (int ii(0); ii < nrows; ++ii) {
	msg << "    ";
	for (int jj(0); jj < ncolumns; ++jj) {
	  if (delta.coeff(ii, jj) <= precision) {
	    if (delta.coeff(ii, jj) < halfmax) {
	      msg << ".";
	    }
	    else {
	      msg << "o";
	    }
	  }
	  else if (delta.coeff(ii, jj) >= tenprecision) {
	    msg << "#";
	  }
	  else {
	    msg << "*";
	  }
	}
	msg << "\n";
      }
  
      return maxdelta <= precision;
    }


    bool check_vector(char const * name,
		      jspace::Vector const & want,
		      jspace::Vector const & have,
		      double precision,
		      std::ostream & msg)
    {
      int const nelems(want.size());
      if (nelems != have.size()) {
	msg << "check_vector(" << name << ") size mismatch: have " << have.size()
	    << " elements but want " << nelems << "\n";
	return false;
      }
  
      precision = fabs(precision);
      double maxdelta(0);
      jspace::Vector delta(nelems);
      for (int ii(0); ii < nelems; ++ii) {
	delta.coeffRef(ii) = fabs(smart_delta(have[ii], want[ii]));
	if (delta.coeff(ii) > precision) {
	  maxdelta = delta.coeff(ii);
	}
      }
      double const halfmax(0.5 * maxdelta);
      double const tenprecision(10 * precision);
  
      if (maxdelta <= precision) {
	msg << "check_vector(" << name << ") OK\n";
      }
      else {
	msg << "check_vector(" << name << ") FAILED\n";
      }
      msg << "  precision = " << precision << "\n"
	  << "  maxdelta = " << maxdelta << "\n";
      pretty_print(delta, msg, "  delta", "    ");
      msg << "  error pattern\n    ";
      for (int ii(0); ii < nelems; ++ii) {
	if (delta.coeff(ii) <= precision) {
	  if (delta.coeff(ii) < halfmax) {
	    msg << ".";
	  }
	  else {
	    msg << "o";
	  }
	}
	else if (delta.coeff(ii) >= tenprecision) {
	  msg << "#";
	}
	else {
	  msg << "*";
	}
      }
      msg << "\n";
  
      return maxdelta <= precision;
    }


    bool check_vector(char const * name,
		      std::vector<double> const & want,
		      std::vector<double> const & have,
		      double precision,
		      std::ostream & msg)
    {
      jspace::Vector ww(want.size());
      memcpy(ww.data(), &want[0], want.size() * sizeof(double));
      jspace::Vector hh(have.size());
      memcpy(hh.data(), &have[0], have.size() * sizeof(double));
      return check_vector(name, ww, hh, precision, msg);
    }
    
    
    std::string create_tmpfile(char const * fname_template, char const * contents) throw(runtime_error)
    {
      if (strlen(fname_template) >= 64) {
	throw runtime_error("create_tmpfile(): fname_template is too long (max 63 characters)");
      }
      
      static char tmpname[64];
      memset(tmpname, '\0', 64);
      strncpy(tmpname, fname_template, 63);
      int const tmpfd(mkstemp(tmpname));
      if (-1 == tmpfd) {
	throw runtime_error("create_tmpfile(): mkstemp(): " + string(strerror(errno)));
      }
      
      size_t const len(strlen(contents));
      if (static_cast<ssize_t>(len) != write(tmpfd, contents, len)) {
	throw runtime_error("create_tmpfile(): write(): " + string(strerror(errno)));
      }
      close(tmpfd);
      
      string result(tmpname);
      return result;
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
	
	static int const buflen(32);
	static char buf[buflen];
	memset(buf, 0, sizeof(buf));
	
	if (vecmode) {
	  if ( ! prefix.empty())
	    os << prefix;
	  for (int ir(0); ir < mm.rows(); ++ir) {
#ifndef WIN32
	    if (isinf(mm.coeff(ir, 0))) {
	      snprintf(buf, buflen-1, " inf    ");
	    }
	    else if (isnan(mm.coeff(ir, 0))) {
	      snprintf(buf, buflen-1, " nan    ");
	    }
	    else if (fabs(fmod(mm.coeff(ir, 0), 1)) < 1e-6) {
	      snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(mm.coeff(ir, 0))));
	    }
	    else {
	      snprintf(buf, buflen-1, "% 6.4f  ", mm.coeff(ir, 0));
	    }
#else // WIN32
	    sprintf_s(buf, buflen-1, "% 6.4f  ", mm.coeff(ir, 0));
#endif // WIN32
	    os << buf;
	  }
	  os << nlornot;
	  
	}
	else {

	  for (int ir(0); ir < mm.rows(); ++ir) {
	    if ( ! prefix.empty())
	      os << prefix;
	    for (int ic(0); ic < mm.cols(); ++ic) {
#ifndef WIN32
	      if (isinf(mm.coeff(ir, ic))) {
		snprintf(buf, buflen-1, " inf    ");
	      }
	      else if (isnan(mm.coeff(ir, ic))) {
		snprintf(buf, buflen-1, " nan    ");
	      }
	      else if (fabs(fmod(mm.coeff(ir, ic), 1)) < 1e-6) {
		snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(mm.coeff(ir, ic))));
	      }
	      else {
		snprintf(buf, buflen-1, "% 6.4f  ", mm.coeff(ir, ic));
	      }
#else // WIN32
	      sprintf_s(buf, buflen-1, "% 6.4f  ", mm.coeff(ir, ic));
#endif // WIN32
	      os << buf;
	    }
	    os << nlornot;
	  }
	  
	}
      }
    }
    
    
    bool equal(jspace::Matrix const & lhs, jspace::Matrix const & rhs, double precision)
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
    
    
    bool equal(jspace::Quaternion const & lhs, jspace::Quaternion const & rhs, double precision)
    {
      return equal(lhs.coeffs(), rhs.coeffs(), precision);
    }
    
  }
}
