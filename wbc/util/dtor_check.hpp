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

#include <string>

namespace wbc {

  /**
     Utility for hunting double-delete bugs. Add one of these to a
     class that gets deleted more than once, and in its destructor
     call dtor_check::check() with the this pointer. The first time it
     gets called, it remembers the stacktrace, and the second time it
     will dump the stack trace and cause a segmentation fault (on
     purpose, so you can easily inspect things with a debugger).
  */
  struct dtor_check {
    dtor_check();
    
    /**
       Verify that a given pointer has not yet been passed to
       delete. Prints the stack trace at the *previous* deletion and
       then causes a segfault. Very useful for finding who deleted us
       first, instead of just seeing the error at the second deletion.

       \note Only seems to work reliable inside virtual
       destructors. See tests/testDtorCheck.cpp for an example.
     */
    void check(void * that);
    
    void * previous_that;
    std::string previous_bt;
  };
  
}
