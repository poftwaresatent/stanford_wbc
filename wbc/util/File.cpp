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
   \file File.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2008 Roland Philippsen, released under a BSD license.
*/

#include "File.hpp"

extern "C" {
#include <errno.h>
#include <string.h>
}

namespace wbc {

  File::
  File(char const * path, char const * mode) throw (std::runtime_error)
    : stream(fopen(path, mode))
  {
    if (0 == stream)
      throw runtime_error(string("fopen(") + path + ", " + mode +"): " + strerror(errno));
    clearerr(stream);
  }
  
  
  File::
  ~File()
  {
    if (0 != stream)
      fclose(stream);
  }

}
