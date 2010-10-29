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

#ifndef WBCNET_LOG_HPP
#define WBCNET_LOG_HPP

/**
   \file log.hpp
   \brief Wrapper around log4cxx.

   This file does compile-time switching on the preprocessor symbols
   DISABLE_LOGGING and HAVE_LOG4CXX in order to achieve the following:
   - If HAVE_LOG4CXX is defined, and DISABLE_LOGGING is not defined,
     then provide logging services through the excellent Apache log4cxx
     library.
   - If neither HAVE_LOG4CXX nor DISABLE_LOGGING are defined, then
     provide very rudimentary logging services that syntactically
     resemble log4cxx
   - If DISABLE_LOGGING is defined, then provide empty wrappers for
     the logging service such that you do not have to insert
     conditional compilation in your code whenever you use logging.
     
   \note Mixing inclusion of this header with log4cxx headers can lead
   to compilation errors when someone alse tries to compile with
   DISABLE_LOGGING, so it is recommended that you always go through
   this header.
     
   The interface provided by the log4cxx wrappers in the case of
   DISABLE_LOGGING or undefined HAVE_LOG4CXX is a subset of the
   log4cxx capabilities:
   - LOG_TRACE(), LOG_DEBUG(), LOG_INFO(), LOG_WARN(), LOG_ERROR(),
     and LOG_FATAL(), which are analoguous to the LOG4CXX_TRACE() etc
     macros.
   - wbcnet::get_root_logger() and wbcnet::get_logger() allow you to
     get logger instances, just like log4cxx::Logger::getRootLogger()
     and log4cxx::Logger::getLogger()
   - You can setLevel() on a logger, using
     e.g. log4cxx::Level::getDebug() (but on fake loggers, these have
     no effect)

   There also is a configure_logging() function which you should use
   to configure the log4cxx system.
*/


namespace wbcnet {

  /**
     Configure the logging system. Has no effect unless HAVE_LOG4CXX
     is defined and DISABLE_LOGGING is undefined.

     Specify a non-null \c force_config_file if you want to override
     the search order. In that case, it will try to load it using \c
     log4cxx::PropertyConfigurator::configure() and \c
     log4cxx::xml::DOMConfigurator::configure().
     
     If \c force_config_file is null (the default), then the following
     directories are searched until the first match is found:
     - \c ${WBC_HOME} (if defined)
     - \c ${HOME}/.wbc (if defined)
     - \c /etc/wbc
     
     In each of these directories, the following filenames are tried:
     - \c log4cxx.properties, \c log4j.properties, \c log.properties
       are read using \c log4cxx::PropertyConfigurator::configure()
     - \c log4cxx.xml, \c log4j.xml, \c log.xml are read using \c
       log4cxx::xml::DOMConfigurator::configure()

     If no configuration files are found, or the one given as \c
     force_config_file did not work, then a default configuration is
     chosen:
     - conversion pattern "%d %-5p %c - %m%n"
     - a single console appender
     - log level WARN
  */
  void configure_logging(char const * force_config_file = 0);
  
  
  /**
     Manually set the log level of the root logger:
     <table border="1">
     <tr><th> verbosity </th><th> log level          </th></tr>
     <tr><td>        <0 </td><td> <em>unchanged</em> </td></tr>
     <tr><td>         0 </td><td>               WARN </td></tr>
     <tr><td>         1 </td><td>               INFO </td></tr>
     <tr><td>         2 </td><td>              DEBUG </td></tr>
     <tr><td>       >=3 </td><td>              TRACE </td></tr>
     </table>
   */
  void manual_logging_verbosity(int verbosity);
  
}

#ifdef DISABLE_LOGGING

//////////////////////////////////////////////////
// The user wants no logging, provide definitions that will create no
// runtime overhead.

# include <wbcnet/imp/fake_log.hpp>

# define LOG_TRACE(logger, arg) /* all logging disabled */
# define LOG_DEBUG(logger, arg) /* all logging disabled */
# define LOG_INFO(logger, arg)  /* all logging disabled */
# define LOG_WARN(logger, arg)  /* all logging disabled */
# define LOG_ERROR(logger, arg) /* all logging disabled */
# define LOG_FATAL(logger, arg) /* all logging disabled */

#else // DISABLE_LOGGING

# ifdef HAVE_LOG4CXX

//////////////////////////////////////////////////
// Logging is enabled, and log4cxx was found --- great, let's just
// wrap it into a thin layer.

#  include <log4cxx/logger.h>

namespace wbcnet {
  using log4cxx::Level;
  typedef log4cxx::LoggerPtr logger_t;
  inline logger_t get_root_logger() { return log4cxx::Logger::getRootLogger(); }
  template<typename string_t> logger_t get_logger(string_t const & name) { return log4cxx::Logger::getLogger(name); }
}

#  define LOG_TRACE(logger, arg) LOG4CXX_TRACE(logger, arg)
#  define LOG_DEBUG(logger, arg) LOG4CXX_DEBUG(logger, arg)
#  define LOG_INFO(logger, arg)  LOG4CXX_INFO(logger, arg)
#  define LOG_WARN(logger, arg)  LOG4CXX_WARN(logger, arg)
#  define LOG_ERROR(logger, arg) LOG4CXX_ERROR(logger, arg)
#  define LOG_FATAL(logger, arg) LOG4CXX_FATAL(logger, arg)

# else // HAVE_LOG4CXX

//////////////////////////////////////////////////
// Unfortunately, log4cxx is not available --- provide a simplistic
// implementations of the logging mechanisms that only prints messages
// of level warning and upwards.

#  include <wbcnet/imp/fake_log.hpp>

// XXXX to do: could use __PRETTY_FUNCTION__ here and also in log4cxx
// wrapper macros in order to automatically insert the fully qualified
// function name.

#  define LOG_TRACE(logger, arg) /* LOG_TRACE disabled */
#  define LOG_DEBUG(logger, arg) /* LOG_DEBUG disabled */
#  define LOG_INFO(logger, arg)  /* LOG_INFO disabled */
#  define LOG_WARN(logger, arg)  { std::cerr<<"WARN "<<logger.name<<": "<<arg<<"\n"; }
#  define LOG_ERROR(logger, arg) { std::cerr<<"ERROR "<<logger.name<<": "<<arg<<"\n"; }
#  define LOG_FATAL(logger, arg) { std::cerr<<"FATAL "<<logger.name<<": "<<arg<<"\n"; }

# endif // HAVE_LOG4CXX

#endif // DISABLE_LOGGING

#endif // WBCNET_LOG_HPP
