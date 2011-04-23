/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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
   \file dynsim.cpp
   \author Roland Philippsen
*/

#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>

#include <jspace/tao_dump.hpp>
//#include <jspace/tao_util.hpp>
#include <jspace/test/sai_brep.hpp>
#include <jspace/test/sai_brep_parser.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <err.h>
#include <stdlib.h>

using namespace std;


static jspace::tao_tree_info_s * tao_tree(0);


static void cleanup(void)
{
  delete tao_tree;
}


/** \note This implementation assumes that all joints are 1 DOF. */
static void get_state(jspace::tao_tree_info_s const * tree, double * pos, double * vel)
{
  typedef jspace::tao_tree_info_s::node_info_t::const_iterator it_t;
  it_t const iend(tree->info.end());
  for (it_t ii(tree->info.begin()); ii != iend; ++ii) {
    ii->joint->getQ(pos++);
    ii->joint->getDQ(vel++);
  }
}


/** \note This implementation assumes that all joints are 1 DOF. */
static void set_state(jspace::tao_tree_info_s * tree, double const * pos, double const * vel)
{
  typedef jspace::tao_tree_info_s::node_info_t::iterator it_t;
  it_t const iend(tree->info.end());
  for (it_t ii(tree->info.begin()); ii != iend; ++ii) {
    ii->joint->setQ(pos++);
    ii->joint->setDQ(vel++);
    ii->joint->zeroDDQ();
  }
}


/** \note This implementation assumes that all joints are 1 DOF. */
static void set_tau(jspace::tao_tree_info_s * tree, double const * tau)
{
  typedef jspace::tao_tree_info_s::node_info_t::iterator it_t;
  it_t const iend(tree->info.end());
  for (it_t ii(tree->info.begin()); ii != iend; ++ii) {
    ii->joint->setTau(tau++);
  }
}


static void str2vec(std::string const & str, vector<double> & vec)
{
  vec.clear();
  istringstream is(str);
  double val;
  while (is >> val) {
    vec.push_back(val);
  }
}


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup)) {
    err(EXIT_FAILURE, "atexit()");
  }
  
  //////////////////////////////////////////////////
  // parse options
  
  string infname("-");
  string outfname("-");
  string saifname("robot.xml");
  double timestep(1e-3);
  size_t nsubsteps(10);
  vector<double> state[2];
  vector<double> & position(state[0]);
  vector<double> & velocity(state[1]);
  int verbosity(0);
  for (int iopt(1); iopt < argc; ++iopt) {
    string const opt(argv[iopt]);
    if ("-i" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-i requires an argument (use -h for some help)");
      }
      infname = argv[iopt];
    }
    else if ("-o" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-o requires an argument (use -h for some help)");
      }
      outfname = argv[iopt];
    }
    else if ("-s" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-s requires an argument (use -h for some help)");
      }
      saifname = argv[iopt];
    }
    else if ("-t" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-t requires an argument (use -h for some help)");
      }
      if (1 != sscanf(argv[iopt], "%lf", &timestep)) {
	err(EXIT_FAILURE, "sscanf(`%s'...)", argv[iopt]);
      }
      timestep *= 1e-3;
    }
    else if ("-n" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-n requires an argument (use -h for some help)");
      }
      if (1 != sscanf(argv[iopt], "%zu", &nsubsteps)) {
	err(EXIT_FAILURE, "sscanf(`%s'...)", argv[iopt]);
      }
      if (0 == nsubsteps) {
	errx(EXIT_FAILURE, "nsubsteps must be > 0");
      }
    }
    else if ("-v" == opt) {
      ++verbosity;
    }
    else if ("-vv" == opt) {
      verbosity += 2;
    }
    else if ("-vvv" == opt) {
      verbosity += 3;
    }
    else if ("-P" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-P requires an argument (use -h for some help)");
      }
      str2vec(argv[iopt], position);
    }
    else if ("-V" == opt) {
      ++iopt;
      if (iopt >= argc) {
	errx(EXIT_FAILURE, "-V requires an argument (use -h for some help)");
      }
      str2vec(argv[iopt], velocity);
    }
    else if ("-h" == opt) {
      printf("Rigid-body dynamics simulator from stanford-wbc.sf.net\n"
	     "  Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University.\n"
	     "                     All rights reserved.\n"
	     "  Redistribution, use, and modification permitted under the LGPLv3.\n"
	     "\n"
	     "usage [-i infile] [-o outfile] [-s saifile] [-vh]\n"
	     "\n"
	     "  -i  input file name   name of the trajectory definition file\n"
	     "                        (use `-' for stdin, which is the default)\n"
	     "  -o  output file name  name of the predicted torque output file\n"
	     "                        (use `-' for stdout, which is the default)\n"
	     "  -s  SAI XML file name name of the robot description file\n"
	     "                        (default is `robot.xml')\n"
	     "  -t  milliseconds      timestep between samples (input and output) in milliseconds\n"
	     "                        (default is 1ms, i.e. a 1kHz control loop)\n"
	     "  -n  nsubsteps         the number of integration substeps\n"
	     "                        (default is 10)\n"
	     "  -P  startpos          start position (vector of space-delimited numbers)\n"
	     "  -V  startvel          start velocity (vector of space-delimited numbers)\n"
	     "  -v                    verbose mode (multiple times makes it more verbose)\n"
	     "  -h                    this message\n");
      exit(EXIT_SUCCESS);
    }
    else {
      errx(EXIT_FAILURE, "invalid option `%s' (use -h for some help)", argv[iopt]);
    }
  }
  
  if (verbosity > 0) {
    printf("Rigid-body dynamics simulator from stanford-wbc.sf.net\n"
	   "  Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University.\n"
	   "                     All rights reserved.\n"
	   "  Redistribution, use, and modification permitted under the LGPLv3.\n"
	   "input file: %s\n"
	   "output file: %s\n"
	   "robot file: %s\n",
	   infname.c_str(), outfname.c_str(), saifname.c_str());
  }
  
  //////////////////////////////////////////////////
  // set up the TAO tree, initial state, and the file streams
  
  try {
    jspace::test::BRParser brp;
    jspace::test::BranchingRepresentation * brep(brp.parse(saifname));
    tao_tree = brep->createTreeInfo();
    if (verbosity > 0) {
      dump_tao_tree_info(cout, tao_tree, "robot", false);
    }
  }
  catch (exception const & ee) {
    errx(EXIT_FAILURE, "exception: %s", ee.what());
  }
  
  size_t const ndof(tao_tree->info.size());
  
  static char const * param_name[] = { "startpos", "startvel" };
  for (size_t ii(0); ii < 2; ++ii) {
    if (state[ii].size() != ndof) {
      if (state[ii].size() == 0) {
	state[ii].assign(ndof, 0.0);
      }
      else if (state[ii].size() == 1) {
	state[ii].assign(ndof, state[ii][0]);
      }
      else {
	errx(EXIT_FAILURE, "dimension mismatch: %s has %zu entries, but should have %zu (or 0 or 1)",
	     param_name[ii], state[ii].size(), ndof);
      }
    }
  }
  
  ifstream infile;
  istream * is;
  if ("-" == infname) {
    is = &cin;
  }
  else {
    infile.open(infname.c_str());
    if ( ! infile) {
      errx(EXIT_FAILURE, "failed to open `%s' for reading", infname.c_str());
    }
    is = &infile;
  }
  
  ofstream outfile;
  ostream * os;
  if ("-" == outfname) {
    os = &cout;
  }
  else {
    outfile.open(outfname.c_str());
    if ( ! outfile) {
      errx(EXIT_FAILURE, "failed to open `%s' for writing", outfname.c_str());
    }
    os = &outfile;
  }
  
  //////////////////////////////////////////////////
  // main processing loop:

  // Read torques, one line at a time. Compute the corresponding
  // motion of the robot, and write out the resulting positions and
  // velocities.
  
  set_state(tao_tree, &position[0], &velocity[0]);
  taoDynamics::updateTransformation(tao_tree->root);
  
  static deVector3 gravity(0, 0, -9.81); // let's assume we're on Earth
  double const substep_dt(timestep / nsubsteps);
  
  for (size_t lineno(1); *is; ++lineno) {
    
    vector<double> tau;
    {
      string line;
      getline(*is, line);
      istringstream ls(line);
      double value;
      while (ls >> value) {
	tau.push_back(value);
      }
      if (tau.empty()) {
	break;
      }
      if (ndof != tau.size()) {
	errx(EXIT_FAILURE,
	     "%s:%zu: error: expected ndof = %zu entries but got %zu",
	     infname.c_str(), lineno, ndof, tau.size());
      }
    }
    
    for (size_t ii(0); ii < nsubsteps; ++ii) {
      set_tau(tao_tree, &tau[0]);
      taoDynamics::fwdDynamics(tao_tree->root, &gravity);
      taoDynamics::integrate(tao_tree->root, substep_dt);
      taoDynamics::updateTransformation(tao_tree->root);
    }
    
    get_state(tao_tree, &position[0], &velocity[0]);
    for (size_t ii(0); ii < ndof; ++ii) {
      *os << tau[ii] << "  ";
    }
    for (size_t ii(0); ii < ndof; ++ii) {
      *os << position[ii] << "  ";
    }
    for (size_t ii(0); ii < ndof; ++ii) {
      *os << velocity[ii] << "  ";
    }
    *os << "\n";
    
  }

  //////////////////////////////////////////////////
  // give a little hint on how to extract the various parts from the
  // generated output
  
  if (("-" != outfname) || (verbosity > 0)) {
    if (ndof > 1) {
      printf("for printing in gnuplot:\n"
	     "- torque:\n"
	     "  plot '%s' u 0:1 w l t 'joint 0', '%s' u 0:2 w l t 'joint 1', ...\n"
	     "- position:\n"
	     "  plot '%s' u 0:%zu w l t 'joint 0', '%s' u 0:%zu w l t 'joint 1', ...\n"
	     "- velocity:\n"
	     "  plot '%s' u 0:%zu w l t 'joint 0', '%s' u 0:%zu w l t 'joint 1', ...\n",
	     outfname.c_str(), outfname.c_str(),
	     outfname.c_str(), ndof + 1, outfname.c_str(), ndof + 2,
	     outfname.c_str(), 2 * ndof + 1, outfname.c_str(), 2 * ndof + 2);
    }
    else {
      printf("for printing in gnuplot:\n"
	     "- torque:\n"
	     "  plot '%s' u 0:1 w l t 'tau'\n"
	     "- position:\n"
	     "  plot '%s' u 0:%zu w l t 'pos'\n"
	     "- velocity:\n"
	     "  plot '%s' u 0:%zu w l t 'vel'\n",
	     outfname.c_str(),
	     outfname.c_str(), ndof + 1,
	     outfname.c_str(), 2 * ndof + 1);
    }
  }
}
