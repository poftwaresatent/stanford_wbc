/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "StringList.hpp"
#include <wbcnet/log.hpp>
#include <iostream>
#include <cstring>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

using namespace std;

namespace wbcnet {
  
  namespace msg {
    
    StringList::
    StringList(unique_id_t id)
      : Proxy(id),
	tt_nchars(0),
	ascii(0)
    {
      m_header.AddField(new ValuePack<uint16_t, fixed_count>
			("StringList::tt_nchars", true, &tt_nchars, 1));
      m_payload.AddField(new VectorPack("StringList::ascii", true, false, &ascii));
    }
    
    
    proxy_status StringList::
    UnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode)
    {
      ascii.SetNElements(tt_nchars);
      return DoUnpackPayload(buffer, endian_mode);
    }
    
    
    bool StringList::
    operator == (StringList const & rhs) const
    {
      if (this == &rhs)
	return true;
      if (tt_nchars != rhs.tt_nchars)
	return false;
      return 0 == memcmp(ascii.DataPointer(), rhs.ascii.DataPointer(), tt_nchars);
    }
    
    
    void StringList::
    clear()
    {
      tt_nchars = 0;
      ascii.SetNElements(0);
    }
    
    
    bool StringList::
    append(uint8_t const * str)
    {
      if (sizeof(uint8_t) != sizeof(char)) {
	LOG_ERROR (logger,
		       "wbcnet::msg::StringList::append(" << str << "): char is not 8 bits???");
	return false;
      }
      size_t len(strlen((char const *) str));
      uint16_t newtt(tt_nchars + len + 1); // always append a null byte
      if (newtt < tt_nchars) {
	LOG_ERROR (logger, "wbcnet::msg::StringList::append(" << str << "): max size reached");
	return false;
      }
      if ( ! ascii.SetNElements(newtt)) {
	LOG_ERROR (logger,
		       "wbcnet::msg::StringList::append(" << str << "): buffer resize failed");
	return false;
      }
      char * cur(ascii.DataPointer() + tt_nchars);
      memcpy(cur, str, len);
      cur += len;
      *cur = 0;			// append null byte
      tt_nchars = newtt;
      return true;
    }
    
    
    bool StringList::
    append(std::string const & str)
    {
      return append((uint8_t const *) str.c_str());
    }
    
    
    bool StringList::
    extract(std::list<std::string> & strlist) const
    {
      if (sizeof(uint8_t) != sizeof(char)) {
	LOG_ERROR (logger, "wbcnet::msg::StringList::extract(): char is not 8 bits???");
	return false;
      }
      if (0 == tt_nchars)
	return true;
      char const * cur(ascii.DataPointer());
      uint16_t pos(0);
      while (pos < tt_nchars) {
	strlist.push_back(std::string(cur));
	// skip to one beyond the next null byte
	while ((0 != *cur) && (pos < tt_nchars)) {
	  ++cur;
	  ++pos;
	}
	++cur;
	++pos;
      }
      return true;
    }
    
    
    void StringList::
    display(std::ostream & os, string const & prefix) const
    {
      list<string> strlist;
      extract(strlist);
      for (list<string>::const_iterator ii(strlist.begin()); ii != strlist.end(); ++ii)
	os << prefix << *ii << "\n";
    }
    
  }

}
