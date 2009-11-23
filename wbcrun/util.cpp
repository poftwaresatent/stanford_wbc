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

#include "util.hpp"

extern "C" {
#include <errno.h>
#include <string.h>
}

using namespace std;

namespace wbcrun {
  
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
