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
#include <string.h>
#include <errno.h>
#include <stdlib.h>

using namespace std;

namespace jspace {
  namespace test {
    
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

  }
}
