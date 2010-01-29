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

#include "log.hpp"

#ifndef DISABLE_LOGGING
# ifdef HAVE_LOG4CXX
#  include "strutil.hpp"
#  include <log4cxx/propertyconfigurator.h>
#  include <log4cxx/xml/domconfigurator.h>
#  include <log4cxx/logmanager.h>
#  include <log4cxx/patternlayout.h>
#  include <log4cxx/consoleappender.h>
#  include <list>
#  include <stdlib.h>
#  include <sys/types.h>
#  include <dirent.h>

using namespace log4cxx;
using namespace std;

static void fallback_config()
{
  LogManager::getLoggerRepository()->setConfigured(true);
  LoggerPtr root(Logger::getRootLogger());
  static const LogString TTCC_CONVERSION_PATTERN(LOG4CXX_STR("%d %-5p %c - %m%n"));
  LayoutPtr layout(new PatternLayout(TTCC_CONVERSION_PATTERN));
  AppenderPtr appender(new ConsoleAppender(layout));
  root->addAppender(appender);
  root->setLevel(log4cxx::Level::getWarn());
}

# endif // HAVE_LOG4CXX
#endif // DISABLE_LOGGING


namespace wbcnet {
  
  void configure_logging(char const * force_config_file)
  {
    // the only case we implement is when we use log4cxx
#ifndef DISABLE_LOGGING
# ifdef HAVE_LOG4CXX
    
    if (force_config_file) {
      try {
	PropertyConfigurator::configure(force_config_file);
	xml::DOMConfigurator::configure(force_config_file);
	fallback_config();
      }
      catch (...) {
	// ignore
      }
      return;
    }
    
    typedef std::list<std::string> listing_t;
    static listing_t search_directories;
    
    if (search_directories.empty()) {
      if (getenv("WBC_HOME")) {
	listing_t env_tok;
	sfl::tokenize(getenv("WBC_HOME"), ':', env_tok);
	for (listing_t::const_iterator it(env_tok.begin()); it != env_tok.end(); ++it)
	  if ( ! it->empty())
	    search_directories.push_back(*it);
      }
      if (getenv("HOME"))
	search_directories.push_back(string(getenv("HOME")) + "/.wbc");
      search_directories.push_back(string("/etc/wbc/"));
    }
    
    // XXXX potential future extension: suppose we want to be able to
    // stack property definitions, then we should traverse the list in
    // reverse order and not stop after the first match.
    for (listing_t::const_iterator dir(search_directories.begin());
	 dir != search_directories.end(); ++dir) {
      DIR * dirp(opendir(dir->c_str()));
      if ( ! dirp)
	continue;
      
      for (struct dirent * dent(readdir(dirp)); 0 != dent; dent = readdir(dirp)) {
	std::string const name(dent->d_name);
	if (("log4cxx.properties" == name) || ("log4j.properties" == name) || ("log.properties" == name)) {
	  try {
	    PropertyConfigurator::configure(*dir + "/" + name);
	    closedir(dirp);
	    return;
	  }
	  catch (...) {
	    // ignore
	  }
	}
	if (("log4cxx.xml" == name) || ("log4j.xml" == name) || ("log.xml" == name)) {
	  try {
	    xml::DOMConfigurator::configure(*dir + "/" + name);
	    closedir(dirp);
	    return;
	  }
	  catch (...) {
	    // ignore
	  }
	}
      }
      
      closedir(dirp);
    }
    
    try {
      fallback_config();
    }
    catch (...) {
      // ignore
    }
    
# endif // HAVE_LOG4CXX
#endif // DISABLE_LOGGING
  }
  
  
  void manual_logging_verbosity(int verbosity)
  {
#ifndef DISABLE_LOGGING
    if (verbosity > 0) {
      wbcnet::logger_t rootLogger(wbcnet::get_root_logger());
      if (verbosity > 2) {
	if ( ! rootLogger->isTraceEnabled())
	  rootLogger->setLevel(log4cxx::Level::getTrace());
      }
      else if (verbosity > 1) {
	if ( ! rootLogger->isDebugEnabled())
	  rootLogger->setLevel(log4cxx::Level::getDebug());
      }
      else {
	if ( ! rootLogger->isInfoEnabled())
	  rootLogger->setLevel(log4cxx::Level::getInfo());
      }
    }
#endif // DISABLE_LOGGING
  }
  
}
