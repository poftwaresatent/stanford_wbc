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
   \file wbcuser.cpp
   \author Roland Philippsen
*/

#include <wbcrun/UserProcess.hpp>
#include <wbcnet/NetConfig.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <cstring>
#include <err.h>
#include <signal.h>
#include <sys/types.h>

using namespace std;


static wbcnet::logger_t logger(wbcnet::get_logger("user"));
static size_t verbosity(0);
static string communication("mq");
static wbcrun::UserProcess * proc(0);


static void cleanup()
{
  if (proc) {
    LOG_WARN (logger, "cleaning up wbcrun::UserProcess");
    wbcrun::UserProcess::Cleanup();
    delete proc;
  }
}


static void handle(int signum)
{
  errx(SIGTERM == signum ? EXIT_SUCCESS : EXIT_FAILURE,
       "signal %d (%s)",
       signum, sys_siglist[signum]);
}


static void usage(ostream & os)
{
  os << "   -h          help (this message)\n"
     << "   -v          increase verbosity\n"
     << "   -vv         increase verbosity twice (shorthand for -v -v)\n"
     << "   -vvv        increase verbosity three times (shorthand for -v -v -v)\n"
     << "   -c   <mode> specify communication mode (default: \"mq\")\n"
     << "               see -h for a list of available modes\n";
}


static void parse_options(int argc, char ** argv)
{
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || (argv[ii][0] != '-')) {
      usage(cerr);
      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
    }
    
    switch (argv[ii][1]) {
    case 'h':
      usage(cout);
      cout << "\navailable communication methods (-c option):\n";
      wbcnet::NetConfig::Help("  ", cout);
      exit(EXIT_SUCCESS);
    case 'v':
      ++verbosity;
      if ((strlen(argv[ii]) > 2) && ('v' == argv[ii][2]))
	++verbosity;
      if ((strlen(argv[ii]) > 3) && ('v' == argv[ii][3]))
	++verbosity;
      break;
    case 'c':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-c requires an option (see -h for more info)");
      }
      communication = argv[ii];
      break;
    default:
      usage(cerr);
      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
    }
  }
}


static void init()
{
  if (0 != atexit(cleanup))
    err(EXIT_FAILURE, "atexit()");
  if (SIG_ERR == signal(SIGTERM, handle))
    err(EXIT_FAILURE, "signal(SIGTERM, ...)");
  if (SIG_ERR == signal(SIGPIPE, handle))
    err(EXIT_FAILURE, "signal(SIGPIPE, ...)");
  
  wbcnet::configure_logging();
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
  
  wbcnet::NetConfig const * netcfg(wbcnet::NetConfig::Create(communication));
  if ( ! netcfg)
    errx(EXIT_FAILURE, "wbcnet::NetConfig::Create(%s) failed", communication.c_str());
  proc = new wbcrun::UserProcess();
  try {
    proc->Init(*netcfg);
  }
  catch (exception const & ee) {
    delete netcfg;
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
  delete netcfg;
}


int main(int argc, char ** argv)
{
  parse_options(argc, argv);
  init();
  while (proc->Step());
}
