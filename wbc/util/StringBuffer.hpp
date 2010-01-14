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
   \file StringBuffer.hpp Utility for appending raw character chunks into a string.
   \author Roland Philippsen
   \note Originally Copyright (c) 2008 Roland Philippsen, released under a BSD license.
*/

#ifndef WBC_UTIL_STRING_BUFFER_HPP
#define WBC_UTIL_STRING_BUFFER_HPP

#include <string>

#ifdef WIN32
#include "ctype.h"
#else
using std::isspace;
#endif

namespace wbc {

  template<typename _CharT>
  class StringBuffer
  {
  public:
    typedef _CharT char_t;
    typedef std::basic_string<char_t> string_t;
    
    StringBuffer() : m_lastchar(' ') {}
    
    /** append from non-null-terminated character string, eating up
	extra whitespace as we go */
    void Append(const char_t * s, int len) {
      for (int ii(0); ii < len; ++ii) {
	if (isspace(s[ii])) {
	  if (' ' != m_lastchar) {
	    m_string += s[ii];
	    m_lastchar = ' ';
	  }
	  // else skip
	}
	else {
	  m_string += s[ii];
	  m_lastchar = s[ii];
	}
      }
    }
    
    string_t const & GetString() const { return m_string; }
    bool Empty() const { return m_string.empty(); }
    
  protected:
    string_t m_string;
    char_t m_lastchar;
  };

}

#endif // WBC_UTIL_STRING_BUFFER_HPP
