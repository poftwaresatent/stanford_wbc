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
   \file options.cpp
   \author Roland Philippsen
*/

#include "options.hpp"
#include <wbcnet/NetConfig.hpp>
#include <iostream>
#include <sstream>
#include <cstring>
#include <stdlib.h>
#include <err.h>

using namespace std;

namespace {
#ifdef DISABLE_NETWORKING
# warning 'Networking is DISABLED, skipping over networking support code.'
#else // DISABLE_NETWORKING
  static char const * default_communication("mq");
#endif // DISABLE_NETWORKING
  static char const * default_xml_filename("../robospecs/puma.xml");
  static char const * default_motion_type("humanoid");
  static char const * default_brparser_type("sai");
}

namespace wbc {

  options::
  options()
    :
#ifndef DISABLE_NETWORKING
      communication(default_communication),
#endif // DISABLE_NETWORKING
      xml_filename(default_xml_filename),
      motion_type(default_motion_type),
      brparser_type(default_brparser_type),
      verbosity(0),
      timestats_skip(0)
  {
  }


  void options::
  usage(std::ostream & os) const
  {
    os << "\ncommon options:\n"
       << "   -h            help (this message)\n"
       << "   -l            do NOT use legacy XML parser (deprecated: use -b option instead)\n"
       << "   -v            increase verbosity\n"
       << "   -vv           increase verbosity twice (shorthand for -v -v)\n"
       << "   -vvv          increase verbosity three times (shorthand for -v -v -v)\n"
#ifndef DISABLE_NETWORKING
       << "   -c   <mode>   specify communication mode (default: \""
       << default_communication << "\")\n"
#endif // DISABLE_NETWORKING
       << "                   see -h for a list of available modes\n"
       << "   -f   <path>   specify robot XML file (default \"" << default_xml_filename << "\")\n"
       << "   -t   <skip>   dump timing every <skip> iteration, skip=0 (default) disables it\n\n"
       << "   -M   <type>   motion type (default `" << default_motion_type << "')\n"
       << "   -b   <type>   branching representation parser type (default `" << default_brparser_type << "')\n";
    specific_usage(os);
  }
  
  
  void options::
  parse(int argc, char ** argv)
  {
    for (int ii(1); ii < argc; ++ii) {
      if ((strlen(argv[ii]) < 2) || (argv[ii][0] != '-')) {
	usage(cerr);
	errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
      }
      else
	switch (argv[ii][1]) {
	
	case 'h':
	  usage(cout);
#ifndef DISABLE_NETWORKING
	  cout << "\navailable communication methods (-c option):\n";
	  wbcnet::NetConfig::Help("  ", cout);
#endif // DISABLE_NETWORKING
	  exit(EXIT_SUCCESS);
	
	case 'v':
	  ++verbosity;
	  if ((strlen(argv[ii]) > 2) && ('v' == argv[ii][2]))
	    ++verbosity;
	  if ((strlen(argv[ii]) > 3) && ('v' == argv[ii][3]))
	    ++verbosity;
	  break;
	  
	case 'l':
	  brparser_type = "osim";
	  break;
	  
#ifndef DISABLE_NETWORKING
	case 'c':
	  ++ii;
	  if (ii >= argc) {
	    usage(cerr);
	    errx(EXIT_FAILURE, "-c requires an option (see -h for more info)");
	  }
	  communication = argv[ii];
	  break;
#endif // DISABLE_NETWORKING
	
	case 'f':
	  ++ii;
	  if (ii >= argc) {
	    usage(cerr);
	    errx(EXIT_FAILURE, "-f requires a filename argument");
	  }
	  xml_filename = argv[ii];
	  break;
	
	case 't':
	  ++ii;
	  if (ii >= argc) {
	    usage(cerr);
	    errx(EXIT_FAILURE, "-t requires a skip argument");
	  }
	  {
	    istringstream is(argv[ii]);
	    if ( ! (is >> timestats_skip)) {
	      usage(cerr);
	      errx(EXIT_FAILURE, "error reading skip argument '%s' of -t", argv[ii]);
	    }
	  }
	  break;
	
	case 'M':
	  ++ii;
	  if (ii >= argc) {
	    usage(cerr);
	    errx(EXIT_FAILURE, "-M requires a motion type argument");
	  }
	  motion_type = argv[ii];
	  break;
	
	case 'b':
	  ++ii;
	  if (ii >= argc) {
	    usage(cerr);
	    errx(EXIT_FAILURE, "-b requires a branching representation parser type argument");
	  }
	  brparser_type = argv[ii];
	  break;
	
	default:
	  {
	    int const next(unknown_option(argc, argv, ii));
	    if (0 > next) {
	      usage(cerr);
	      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
	    }
	    ii = next;
	  }
	}
    }
  }

}
