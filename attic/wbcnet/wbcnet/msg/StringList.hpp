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

#ifndef WBCNET_MSG_STRING_LIST_HPP
#define WBCNET_MSG_STRING_LIST_HPP

#include <wbcnet/proxy.hpp>
#include <iosfwd>
#include <list>

namespace wbcnet {

  namespace msg {

    class StringList
      : public Proxy
    {
    public:
      explicit StringList(unique_id_t id);
      
      virtual proxy_status UnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode);
      
      bool operator == (StringList const & rhs) const;
      bool operator != (StringList const & rhs) const { return ! (*this == rhs); }
      
      void clear();
      
      /** \return true on success, false on failure. Failures "should"
	  be rare: either char is not 8 bits, or we would overflow the
	  uint16_t max size. */
      bool append(uint8_t const * str);
      
      /** \return true on success, false on failure. Failures "should"
	  be rare: either char is not 8 bits, or we would overflow the
	  uint16_t max size. */
      bool append(std::string const & str);
      
      /** \return true on success, false on failure. Failures "should"
	  be rare: either char is not 8 bits, or we would overflow the
	  uint16_t max size. */
      template<typename iterator_t>
      bool append(iterator_t begin, iterator_t end) {
	for (/**/; begin != end; ++begin)
	  if ( ! append(*begin))
	    return false;
	return true;
      }
      
      /**
	 \return true pretty much always, even if the string list is
	 empty. Only if sizeof(char) is not sizeof(uint8_t) will it
	 return false... and when that happens we'll have to think
	 about proper porting to unicode or so.
       */
      bool extract(std::list<std::string> & strlist) const;
      
      void display(std::ostream & os, std::string const & prefix) const;
      
      // header
      uint16_t tt_nchars;
      
      // payload
      Vector<uint8_t> ascii;
    };
    
  }
  
}

#endif // WBCNET_MSG_STRING_LIST_HPP
