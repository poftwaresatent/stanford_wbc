/*
 * Reflexxes Type I OTG Library
 *
 * Copyright (C) 2010 Reflexxes GmbH. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
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
   \file trjgen.cpp
   \author Roland Philippsen
*/


#include <reflexxes_otg/TypeIOTG.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <err.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

static double tcycle(1e-3);
static double default_value[] = { 0.0, 0.0, 10.0, 3.0, 6.0 };
static vector<double> params[5];
static vector<double> & startpos(params[0]);
static vector<double> & startvel(params[1]);
static vector<double> & goalpos(params[2]);
static vector<double> & maxvel(params[3]);
static vector<double> & maxacc(params[4]);


static void str2vec(std::string const & str, vector<double> & vec)
{
  vec.clear();
  istringstream is(str);
  double val;
  while (is >> val) {
    vec.push_back(val);
  }
}


static void parse_options(int argc, char ** argv)
{
  bool dryrun(false);
  
  for (int iopt(1); iopt < argc; ++iopt) {
    string const opt(argv[iopt]);
    if ("-t" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-t requires an argument (use -h for some help)");
      }
      if ( 1 != sscanf(argv[iopt], "%lf", &tcycle)) {
	err(EXIT_FAILURE, "sscanf(`%s'...)", argv[iopt]);
      }
      tcycle *= 1e-3;
    }
    else if ("-p" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-p requires an argument (use -h for some help)");
      }
      str2vec(argv[iopt], startpos);
    }
    else if ("-v" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-v requires an argument (use -h for some help)");
      }
      str2vec(argv[iopt], startvel);
    }
    else if ("-g" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-g requires an argument (use -h for some help)");
      }
      str2vec(argv[iopt], goalpos);
    }
    else if ("-V" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-V requires an argument (use -h for some help)");
      }
      str2vec(argv[iopt], maxvel);
    }
    else if ("-a" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-a requires an argument (use -h for some help)");
      }
      str2vec(argv[iopt], maxacc);
    }
    else if ("-d" == opt) {
      dryrun = true;
    }
    else if ("-h" == opt) {
      printf("usage [-t tcycle_milliseconds]\n"
	     "      [-p startpos]\n"
	     "      [-v startvel]\n"
	     "      [-g goalpos]\n"
	     "      [-V maxvel]\n"
	     "      [-a maxacc]\n"
	     "      [-hd]\n"
	     "\n"
	     "  All paramater values except tcycle are strings of space-separated\n"
	     "  floating point numbers.  The dimensions of all given parameters must\n"
	     "  match. If you leave parameters unspecified, they will be filled in\n"
	     "  from default values, which are the following:\n"
	     "\n"
	     "    tcycle = 1\n"
	     "    startpos = %g\n"
	     "    startvel = %g\n"
	     "    goalpos = %g\n"
	     "    maxvel = %g\n"
	     "    maxacc = %g\n",
	     default_value[0], default_value[1], default_value[2], default_value[3], default_value[4]);
      exit(EXIT_SUCCESS);
    }
    else {
      errx(EXIT_FAILURE, "invalid option `%s' (use -h for some help)", argv[iopt]);
    }
  }
  
  size_t nmax(0);
  for (size_t ii(0); ii < 5; ++ii) {
    if (params[ii].size() > nmax) {
      nmax = params[ii].size();
    }
  }
  if (0 == nmax) {
    nmax = 1;
  }
  
  static char const * param_name[] = { "startpos", "startvel", "goalpos", "maxvel", "maxacc" };
  
  for (size_t ii(0); ii < 5; ++ii) {
    if (params[ii].size() != nmax) {
      if (params[ii].size() == 0) {
	params[ii].assign(nmax, default_value[ii]);
      }
      else if (params[ii].size() == 1) {
	params[ii].assign(nmax, params[ii][0]);
      }
      else {
	errx(EXIT_FAILURE, "dimension mismatch: %s has %zu entries, but should have %zu (or 0 or 1)",
	     param_name[ii], params[ii].size(), nmax);
      }
    }
  }
  
  if (dryrun) {
    for (size_t ii(0); ii < 5; ++ii) {
      printf("%s:  ", param_name[ii]);
      for (vector<double>::const_iterator jj(params[ii].begin()); jj != params[ii].end(); ++jj) {
	cout <<" " << *jj;
      }
      cout << "\n";
    }
    exit(EXIT_SUCCESS);
  }
}


int main(int argc, char ** argv)
{
  parse_options(argc, argv);
  
  TypeIOTG otg(startpos.size(), tcycle);
  vector<double> cpos(startpos);
  vector<double> cvel(startvel);
  vector<double> npos(startpos.size());
  vector<double> nvel(startpos.size());
  
  // I would like to use:
  //   static vector<bool> selection;
  //   selection.assign(nmax, true);
  // But that ends up creating a temporary std::_Bit_reference*
  // further down, grrr. Probably something to do with vector<bool>
  // being specialized and implemented via bitset, but Torsten's code
  // want full-fledged bool instances.
  bool * selection(new bool[startpos.size()]);
  for (size_t ii(0); ii < startpos.size(); ++ii) {
    selection[ii] = true;
  }
  
  int result(TypeIOTG::OTG_WORKING);
  while (TypeIOTG::OTG_WORKING == result) {
    result = otg.GetNextMotionState_Position(&cpos[0], &cvel[0],
					     &maxvel[0], &maxacc[0],
					     &goalpos[0], selection,
					     &npos[0], &nvel[0]);
    if (0 <= result) {
      for (vector<double>::const_iterator ii(cpos.begin()); ii != cpos.end(); ++ii) {
	cout << *ii << " ";
      }
      for (vector<double>::const_iterator ii(cvel.begin()); ii != cvel.end(); ++ii) {
	cout << *ii << " ";
      }
      cout << "\n";
      swap(cpos, npos);
      swap(cvel, nvel);
    }
  }
  delete[] selection;
  
  if (0 > result) {
    errx(EXIT_FAILURE, "OTG returned error code %d", result);
  }
  
  for (vector<double>::const_iterator ii(cpos.begin()); ii != cpos.end(); ++ii) {
    cout << *ii << " ";
  }
  for (vector<double>::const_iterator ii(cvel.begin()); ii != cvel.end(); ++ii) {
    cout << *ii << " ";
  }
  cout << "\n";
}
