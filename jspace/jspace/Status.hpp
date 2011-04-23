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
   \file jspace/Status.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_STATUS_HPP
#define JSPACE_STATUS_HPP

#include <string>

namespace jspace {
  
  class Status
  {
  public:
    
    /** Default ctor sets \c ok=true and \c errstr="" */
	inline Status(): ok(true), errstr("") {}
    Status(bool ok, std::string const & errstr);
    
    inline operator bool () const { return ok; }
    
    bool ok;
    std::string errstr;
  };
  
}

#endif // JSPACE_STATUS_HPP
