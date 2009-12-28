/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
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
   \file utc.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2008 Roland Philippsen, released under a BSD license.
*/

#include "utc.hpp"

namespace wbc {

#ifndef WIN32  
  std::string get_utc() throw(std::runtime_error)
  {
    static size_t const bufsize(128);
    char buf[bufsize];
    buf[0] = '\0';
    time_t utc;
    if (-1 == time(&utc))
      throw runtime_error(string("wbcrun::get_utc(): time() failed: ") + strerror(errno));
    struct tm utc_tm;
    if (0 == gmtime_r(&utc, &utc_tm))
      throw runtime_error("wbcrun::get_utc(): gmtime_r() failed");
    if (0 == strftime(buf, bufsize-1, "%F %T", &utc_tm))
      throw runtime_error("wbcrun::get_utc(): strftime() failed");
    buf[bufsize-1] = '\0';	// just paranoid
    
    string result(buf);
    return result;
  }
#endif

}
