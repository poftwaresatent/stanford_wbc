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
   \file jspace/vector_util.cpp
   \author Roland Philippsen
*/

#include "vector_util.hpp"
#include <iostream>
#include <string.h>
#include <stdio.h>


namespace std {
  
  ostream & operator << (ostream & os, vector<double> const & rhs)
  {
    if ( ! rhs.empty()) {    
      static int const buflen(32);
      char buf[buflen];
      memset(buf, 0, sizeof(buf));
      for (vector<double>::const_iterator ii(rhs.begin()); ii != rhs.end(); ++ii) {

#ifndef WIN32

	if (isinf(*ii)) {
	  snprintf(buf, buflen-1, " inf    ");
	}
	else if (isnan(*ii)) {
	  snprintf(buf, buflen-1, " nan    ");
	}
	else if (fabs(fmod(*ii, 1)) < 1e-6) {
	  snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(*ii)));
	}
	else {
	  snprintf(buf, buflen-1, "% 6.4f  ", *ii);
	}

#else

	sprintf_s(buf, buflen-1, "% 6.4f  ", *ii);

#endif // WIN32

	os << buf;
      }
    }
    return os;
  }
  
}
