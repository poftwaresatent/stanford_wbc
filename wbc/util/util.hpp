/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#ifndef WBCRUN_UTIL_HPP
#define WBCRUN_UTIL_HPP

#include <list>
#include <string>
#include <stdexcept>

#ifdef WIN32
#include "ctype.h"
#else
using std::isspace;
#endif


extern "C" {
#include <stdio.h>
}

namespace wbcnet {
  class Proxy;
  class Sink;
  class Source;
}

namespace wbcrun {
  
  
  /** Has to be a list because we remove items from it while iterating
      over its elements. */
  typedef std::list<wbcnet::Proxy *> proxylist_t;
  
  
  /** Contains pending outgoing messages, where they are to be sent,
      and how many should be sent. */
  struct outgoing_s {
    outgoing_s(wbcnet::Sink * _sink, int _max_n_snd)
      : sink(_sink), max_n_snd(_max_n_snd) {}
    
    wbcnet::Sink * sink;
    int max_n_snd; /** maximum number of messages to send (use "-1" for unlimited) */
    proxylist_t pending;
    proxylist_t done;
  };
  
  
  /** Contains an incoming channel and how many messages should be
      attempted to be read from it each time. */
  struct incoming_s {
    incoming_s(wbcnet::Source * _source, int _max_n_rcv)
      : source(_source), max_n_rcv(_max_n_rcv) {}
    
    wbcnet::Source * source;
    int max_n_rcv; /** maximum number of messages to receive (use "-1"
		       for unlimited, which creates an infinite loop,
		       so maybe do not use it that way) */
  };


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
  
  
  /**
     Wraps a FILE*, closing it in the destructor.
  */
  class File
  {
  public:
    File(char const * path, char const * mode) throw (std::runtime_error);
    ~File();
    
    FILE* stream;
  };
  
  
#ifndef WIN32  
  std::string get_utc() throw(std::runtime_error);
#endif

}

#endif // WBCRUN_UTIL_HPP
