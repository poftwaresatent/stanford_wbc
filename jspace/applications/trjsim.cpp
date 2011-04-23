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
   \file trjsim.cpp
   \author Roland Philippsen
*/

#include <jspace/Model.hpp>
#include <jspace/tao_dump.hpp>
#include <jspace/test/sai_brep.hpp>
#include <jspace/test/sai_brep_parser.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <err.h>
#include <stdlib.h>

using namespace std;


int main(int argc, char ** argv)
{
  //////////////////////////////////////////////////
  // parse options
  
  string infname("-");
  string outfname("-");
  string saifname("robot.xml");
  double timestep(1e-3);
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
    else if ("-v" == opt) {
      ++verbosity;
    }
    else if ("-vv" == opt) {
      verbosity += 2;
    }
    else if ("-vvv" == opt) {
      verbosity += 3;
    }
    else if ("-h" == opt) {
      printf("Trajectory simulator from stanford-wbc.sf.net\n"
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
	     "  -t  milliseconds      timestep between samples in milliseconds\n"
	     "                        (default is 1ms, i.e. a 1kHz control loop)\n"
	     "  -v                    verbose mode (multiple times makes it more verbose)\n"
	     "  -h                    this message\n");
      exit(EXIT_SUCCESS);
    }
    else {
      errx(EXIT_FAILURE, "invalid option `%s' (use -h for some help)", argv[iopt]);
    }
  }
  
  if (verbosity > 0) {
    printf("Trajectory simulator from stanford-wbc.sf.net\n"
	   "  Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University.\n"
	   "                     All rights reserved.\n"
	   "  Redistribution, use, and modification permitted under the LGPLv3.\n"
	   "input file: %s\n"
	   "output file: %s\n"
	   "robot file: %s\n",
	   infname.c_str(), outfname.c_str(), saifname.c_str());
  }
  
  //////////////////////////////////////////////////
  // set up the joint-space model and the file streams
  
  jspace::Model model;
  try {
    // One day we will replace ye olde SAI XML format with something
    // more human-friendly, but for now we need to jump through some
    // hoops to get a jspace::Model properly initialized.
    jspace::test::BRParser brp;
    jspace::test::BranchingRepresentation * brep(brp.parse(saifname));
    jspace::tao_tree_info_s * tree(brep->createTreeInfo());
    if (verbosity > 0) {
      dump_tao_tree_info(cout, tree, "robot", false);
    }
    if (0 != model.init(tree, 0, &cerr)) {
      throw runtime_error("jspace::Model::init() failed");
    }
  }
  catch (exception const & ee) {
    errx(EXIT_FAILURE, "exception: %s", ee.what());
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

  // Read positions and velocities, one line at a time. Compute the
  // corresponding acceleration and use the jspace model to determine
  // the forces that would be required to produce these
  // accelerations. Write all of that to the output file.
  
  jspace::State prevstate;
  size_t const ndof(model.getNDOF());
  size_t const ndof2(2 * ndof);
  for (size_t lineno(1); *is; ++lineno) {
    
    jspace::State nextstate;
    {
      vector<double> array;
      string line;
      getline(*is, line);
      istringstream ls(line);
      double value;
      while (ls >> value) {
	array.push_back(value);
      }
      if (ndof2 != array.size()) {
	if (0 != prevstate.position_.size()) {
	  break;
	}
	errx(EXIT_FAILURE,
	     "%s:%zu: error: expected 2*ndof = %zu entries but got %zu",
	     infname.c_str(), lineno, ndof2, array.size());
      }
      jspace::convert(&array[0], ndof, nextstate.position_);
      jspace::convert(&array[ndof], ndof, nextstate.velocity_);
    }
    
    if (0 != prevstate.position_.size()) {
      jspace::Vector acc;
      acc = (nextstate.velocity_ - prevstate.velocity_) / timestep;
      
      model.update(prevstate);
      jspace::Matrix aa;
      if ( ! model.getMassInertia(aa)) {
	errx(EXIT_FAILURE, "weird, jspace::Model::getMassInertia() failed");
      }
      jspace::Vector gg;
      if ( ! model.getGravity(gg)) {
	errx(EXIT_FAILURE, "weird, jspace::Model::getGravity() failed");
      }
      
      jspace::Vector tau_nog, tau;
      tau_nog = aa * acc;
      tau = tau_nog + gg;
      
      *os << jspace::pretty_string(prevstate.position_) << "   "
	  << jspace::pretty_string(prevstate.velocity_) << "   "
	  << jspace::pretty_string(acc) << "   "
	  << jspace::pretty_string(tau_nog) << "   "
	  << jspace::pretty_string(tau) << "\n";
    }
    
    prevstate = nextstate;
  }

  //////////////////////////////////////////////////
  // give a little hint on how to extract the various parts from the
  // generated output
  
  if (("-" != outfname) || (verbosity > 0)) {
    if (ndof > 1) {
      printf("for printing in gnuplot:\n"
	     "- position:\n"
	     "  plot '%s' u 0:1 w l t 'joint 0', '%s' u 0:2 w l t 'joint 1', ...\n"
	     "- velocity:\n"
	     "  plot '%s' u 0:%zu w l t 'joint 0', '%s' u 0:%zu w l t 'joint 1', ...\n"
	     "- acceleration:\n"
	     "  plot '%s' u 0:%zu w l t 'joint 0', '%s' u 0:%zu w l t 'joint 1', ...\n"
	     "- torque without gravity compensation:\n"
	     "  plot '%s' u 0:%zu w l t 'joint 0', '%s' u 0:%zu w l t 'joint 1', ...\n"
	     "- torque with gravity compensation:\n"
	     "  plot '%s' u 0:%zu w l t 'joint 0', '%s' u 0:%zu w l t 'joint 1', ...\n",
	     outfname.c_str(), outfname.c_str(),
	     outfname.c_str(), ndof + 1, outfname.c_str(), ndof + 2,
	     outfname.c_str(), 2 * ndof + 1, outfname.c_str(), 2 * ndof + 2,
	     outfname.c_str(), 3 * ndof + 1, outfname.c_str(), 3 * ndof + 2,
	     outfname.c_str(), 4 * ndof + 1, outfname.c_str(), 4 * ndof + 2);
    }
    else {
      printf("for printing in gnuplot:\n"
	     "- position:\n"
	     "  plot '%s' u 0:1 w l t 'pos'\n"
	     "- velocity:\n"
	     "  plot '%s' u 0:%zu w l t 'vel'\n"
	     "- acceleration:\n"
	     "  plot '%s' u 0:%zu w l t 'acc'\n"
	     "- torque without gravity compensation:\n"
	     "  plot '%s' u 0:%zu w l t 'tau'\n"
	     "- torque with gravity compensation:\n"
	     "  plot '%s' u 0:%zu w l t 't+g'\n",
	     outfname.c_str(),
	     outfname.c_str(), ndof + 1,
	     outfname.c_str(), 2 * ndof + 1,
	     outfname.c_str(), 3 * ndof + 1,
	     outfname.c_str(), 4 * ndof + 1);
    }
  }
}
