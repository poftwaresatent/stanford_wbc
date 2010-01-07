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

#ifndef WBCNET_FAKE_LOG_HPP
#define WBCNET_FAKE_LOG_HPP

// NOTE: Only include this header if you do not have log4cxx or if you
// want to explicitly disable logging. Actually, you should not
// include this header, but wbcnet/log.hpp which does the necessary
// compile-time switching.


#ifndef DISABLE_LOGGING
# include <string>
# include <iostream>
#endif // DISABLE_LOGGING


namespace wbcnet {
  
  /**
     Looks like a subset of log4cxx::Logger, but does not do
     anything. With the appropriate flags, the compiler will
     optimize-out most if not all of this.
  */
  struct fake_logger {

    // fake names and actual instances are only needed for fallback logger
#ifndef DISABLE_LOGGING

    explicit fake_logger(std::string const & _name): name(_name) {}

    std::string const name;

    fake_logger * operator -> () { return this; }

#endif // DISABLE_LOGGING
    
    static bool isTraceEnabled() { return false; }
    static bool isDebugEnabled() { return false; }
    static bool isInfoEnabled() { return false; }
    static bool isWarnEnabled() { return true; }
    static bool isErrorEnabled() { return true; }
    static bool isFatalEnabled() { return true; }
    
    static void setLevel(int) {}
  };
  
#ifdef DISABLE_LOGGING
  
  static fake_logger __fake_logger;
  typedef fake_logger * logger_t;
  
  static logger_t get_root_logger() { return &__fake_logger; }
  template<typename string_t> logger_t get_logger(string_t const & name) { return &__fake_logger; }
  
#else // DISABLE_LOGGING
  
  typedef fake_logger logger_t;
  static logger_t get_root_logger() { return fake_logger(""); }
  template<typename string_t> logger_t get_logger(string_t const & name) { fake_logger foo(name); return foo; }
  
#endif // DISABLE_LOGGING
  
}

namespace wbcnet {

  /**
     Looks like a subset of log4cxx::Level, but does not do
     anything. With the appropriate flags, the compiler will
     optimize-out most if not all of this.
  */
  struct Level {
    static int getTrace() { return 0; }
    static int getDebug() { return 0; }
    static int getInfo() { return 0; }
    static int getWarn() { return 0; }
    static int getError() { return 0; }
    static int getFatal() { return 0; }
  };
}

#endif // WBCNET_FAKE_LOG_HPP
