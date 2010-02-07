/*
 * Copyright (c) 2010 Stanford University
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
   \file robot-bridge.cpp
   \author Roland Philippsen
*/

#include <wbc/core/Plugin.hpp>
#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/bin/builtin.hpp>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <wbcnet/DelayHistogram.hpp>
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <sstream>
#include <sys/time.h>
#include <err.h>
#include <signal.h>
#include <stdlib.h>
#include <vector>

using namespace std;
using namespace wbc;


static void usage(ostream & os);
static void parse_options(int argc, char ** argv);
static void init();
static void cleanup();
static void handle(int signum);
static void loop();

static int pskip(1);
static int ndof(30);
static string in_robot_type("fake:100000:10");
static vector<string> out_robot_type;
static bool verbose(false);
static bool skip_first_half(false);

static size_t active_out_idx(0);
static vector<BidirectionalRobotAPI *> out_robot;
static RobotAPI * in_robot;
static SAIVector * jointPositions;
static SAIVector * jointVelocities;
static SAIMatrix * contactForces;
static vector<SAIVector *> torques;
static wbc::Extensions * extensions;

static double msfloor;
static double msceil;
static int nbins;
static int oskip;
static wbcnet::DelayHistogram * dhist;


int main(int argc, char ** argv)
{
  atexit(cleanup);
  if (SIG_ERR == signal(SIGTERM, handle))
    err(EXIT_FAILURE, "signal(%s)", sys_siglist[SIGTERM]);
  if (SIG_ERR == signal(SIGPIPE, handle))
    err(EXIT_FAILURE, "signal(%s)", sys_siglist[SIGPIPE]);
  
  wbcnet::configure_logging();
  parse_options(argc, argv);
  init();
  loop();
}


void loop()
{
  for (size_t iteration(1); true; ++iteration) {
    
    //////////////////////////////////////////////////
    // first half
    
    if (skip_first_half) {
      if (verbose)
	warnx("skipping first half iteration");
      if ((pskip > 0) && (0 == (iteration % pskip)))
	cout << "======== iteration " << iteration << "\n"
	     << ">>>>>>>> no output (skipping first half)\n";
      skip_first_half = false;
    }
    else {
      
      if (verbose)
	warnx("reading sensors");
      
      if (1 != iteration)
      	dhist->Start(0);
      timeval acquisition_time;
      if ( ! in_robot->readSensors(*jointPositions, *jointVelocities, acquisition_time,
				   contactForces))
	errx(EXIT_FAILURE, "in_robot->readSensors() failed");
      if (1 != iteration)
      	dhist->Stop(0);
      
      if ((pskip > 0) && (0 == (iteration % pskip))) {
	cout << "======== iteration " << iteration << "\n"
	     << ">>>>>>>>\n"
	     << ">> act pos:";
	for (int ii(0); ii < ndof; ++ii)
	  cout << " " << (*jointPositions)[ii];
	cout << "\n>> act vel:";
	for (int ii(0); ii < ndof; ++ii)
	  cout << " " << (*jointVelocities)[ii];
	cout << "\n>> act forces:";
	if (0 == contactForces->size())
	  cout << " NONE\n";
	else {
	  for (int icol(0); icol < contactForces->column(); ++icol) {
	    cout << " [";
	    for (int irow(0); irow < contactForces->row(); ++irow)
	      cout << " " << contactForces->elementAt(irow, icol);
	    cout << " ]";
	  }
	  cout << "\n";
	}
      }
      
      if (verbose)
	warnx("writing sensors");
      
      for (size_t ii(0); ii < out_robot.size(); ++ii) {
	if (1 != iteration)
	  dhist->Start(3 + 2 * ii);
	if ( ! out_robot[ii]->writeSensors(*jointPositions, *jointVelocities, contactForces))
	  errx(EXIT_FAILURE, "out_robot[%zu]->writeSensors() failed, spec = %s",
	       ii, out_robot_type[ii].c_str());
	if (1 != iteration)
	  dhist->Stop(3 + 2 * ii);
      }
    }
    
    //////////////////////////////////////////////////
    // second half
    
    if (verbose)
      warnx("receiving torques and hip command");

    for (size_t ii(0); ii < out_robot_type.size(); ++ii) {
      if (1 != iteration)
      	dhist->Start(2 + 2 * ii);
      if ( ! out_robot[ii]->readCommand(*torques[ii]))
	errx(EXIT_FAILURE, "out_robot[%zu]->readTorques() failed, spec = %s",
	     ii, out_robot_type[ii].c_str());
      if (1 != iteration)
      	dhist->Stop(2 + 2 * ii);
    }
    
    if ((pskip > 0) && (0 == (iteration % pskip))) {
      cout << "<<<<<<<<\n";
      for (size_t ii(0); ii < out_robot_type.size(); ++ii) {
	cout << "<< cmd torques [" << ii << "]:";
	for (int jj(0); jj < ndof; ++jj)
	  cout << " " << (*torques[ii])[jj];
	cout << "\n";
      }
    }
    
    if (verbose)
      warnx("writing torques and hip command");
    
    if (1 != iteration)
      dhist->Start(1);
    if ( ! in_robot->writeCommand(*torques[active_out_idx]))
      errx(EXIT_FAILURE, "in_robot->writeTorques() failed, active_out_idx = %zu, spec = %s",
	   active_out_idx, out_robot_type[active_out_idx].c_str());
    if (1 != iteration)
      dhist->Stop(1);
    
    if ((1 != iteration) && (0 == iteration % oskip)) {
      cout << "##################################################\n"
    	   << "bridge delay histograms:\n";
      dhist->DumpTable(stdout);
      cout << "\nin_robot stats:\n";
      in_robot->dumpStats("  ", cout);
      for (size_t ii(0); ii < out_robot.size(); ++ii) {
    	cout << "\nout_robot [" << ii << "] stats:\n";
    	out_robot[ii]->dumpStats("  ", cout);
      }
    }
  }
}


void cleanup()
{
  if (in_robot)
    in_robot->shutdown();
  delete in_robot;
  for (size_t ii(0); ii < out_robot.size(); ++ii) {
    out_robot[ii]->shutdown();
    delete out_robot[ii];
  }
  delete dhist;
  for (size_t ii(0); ii < out_robot.size(); ++ii)
    delete torques[ii];
  delete contactForces;
  delete jointVelocities;
  delete jointPositions;
}


void usage(ostream & os)
{
  os << "options:\n"
     << "   -h          help (this message)\n"
    ////     << "   -H          help on registered RobotAPI factories\n"
     << "   -v          verbose mode (debug messages)\n"
     << "   -P          skip first half of first iteration (to pretend we're a servo)\n"
     << "   -i <spec>   input RobotAPI spec for readSensors and writeTorques (default fake:100000:10)\n"
     << "   -o <spec>   output RobotAPI spec for writeSensor and readTorques (default fake:100000:10)\n"
     << "               NOTE: you can specify more than one output robot by repeating -o\n"
     << "   -p <skip>   print status every skip loops (default = 1, so every loop)\n"
     << "   -n <NDOF>   override NDOF (default == 30)\n"
     << "   -d msfloor:msceil:nbins:oskip\n"
     << "               measure timing histograms, dumped every oskip iterations\n"
     << "               default msfloor=10, msceil=20, nbins=20, oskip=100\n";
}


void parse_options(int argc, char ** argv)
{
  msfloor = 10;
  msceil = 20;
  nbins = 20;
  oskip = 100;
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      cerr << argv[0] << ": problem with option '" << argv[ii] << "'\n";
      usage(cerr);
      exit(EXIT_FAILURE);
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(cout);
	exit(EXIT_SUCCESS);
	
      // case 'H':
      // 	extensions->robot_registry->dumpAll("  ", cout);
      // 	exit(EXIT_SUCCESS);
	
      case 'v':
	verbose = true;
	break;
	
      case 'P':			// upper-case
	skip_first_half = true;
	break;
	
      case 'i':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -i requires input robot spec\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	in_robot_type = argv[ii];
 	break;
	
      case 'o':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -o requires output robot spec\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	out_robot_type.push_back(argv[ii]);
 	break;
	
      case 'p':
      	++ii;
      	if (ii >= argc)
      	  errx(EXIT_FAILURE, "-p requires a skip argument");
      	{
      	  istringstream is(argv[ii]);
      	  if ( ! (is >> pskip))
      	    errx(EXIT_FAILURE, "error reading skip argument '%s'", argv[ii]);
      	}
      	break;
	
      case 'n':
	++ii;
	if (ii >= argc)
	  errx(EXIT_FAILURE, "-n requires NDOF argument");
	{
	  istringstream is(argv[ii]);
	  if ( ! (is >> ndof))
	    errx(EXIT_FAILURE, "error reading NDOF argument '%s'", argv[ii]);
	}
	if (0 >= ndof)
	  errx(EXIT_FAILURE, "you requested %d NDOF, that makes no sense", ndof);	  
	break;
	
      case 'd':
      	{
      	  ++ii;
      	  if (ii >= argc)
      	    errx(EXIT_FAILURE, "-d requires an argument");
      	  vector<string> tlist;
      	  sfl::tokenize(argv[ii], ':', tlist);
      	  sfl::token_to(tlist, 0, msfloor);
      	  sfl::token_to(tlist, 1, msceil);
      	  sfl::token_to(tlist, 2, nbins);
      	  sfl::token_to(tlist, 3, oskip);
      	  if (nbins < 1)
      	    errx(EXIT_FAILURE, "invalid nbins %d for -d argument", nbins);
      	  if (oskip < 1)
      	    errx(EXIT_FAILURE, "invalid oskip %d for -d argument", oskip);
      	}
      	break;
	
      default:
	cerr << argv[0] << ": invalid option '" << argv[ii] << "'\n";
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
}


void handle(int signum)
{
  errx(EXIT_FAILURE, "caught %s", sys_siglist[signum]);
}


void init()
{
  wbcnet::manual_logging_verbosity(verbose ? 0 : 2);
  
  try {
    extensions = wbc::load_extensions(0);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  if (out_robot_type.empty())
    out_robot_type.push_back("fake:100000:10");
  for (size_t ii(0); ii < out_robot_type.size(); ++ii) {
    BidirectionalRobotAPI * out(extensions->robot_registry->parseCreateBidirectional(out_robot_type[ii], NULL));
    if ( ! out)
      errx(EXIT_FAILURE, "invalid out_robot_type %s", out_robot_type[ii].c_str());
    out_robot.push_back(out);
  }

  in_robot = extensions->robot_registry->parseCreate(in_robot_type, NULL);
  if ( ! in_robot)
    errx(EXIT_FAILURE, "invalid in_robot_type %s", in_robot_type.c_str());
  
  jointPositions = new SAIVector(ndof);
  jointVelocities = new SAIVector(ndof);
  contactForces = new SAIMatrix();
  
  for (size_t ii(0); ii < out_robot_type.size(); ++ii)
    torques.push_back(new SAIVector(ndof));
  
  size_t const nslots(2 + 2 * out_robot_type.size());
  dhist = new wbcnet::DelayHistogram(nslots, nbins, msfloor, msceil);
  dhist->SetName(0, "read from " + in_robot_type);
  dhist->SetName(1, "write to " + in_robot_type);
  for (size_t ii(0); ii < out_robot_type.size(); ++ii) {
    dhist->SetName(2 + 2 * ii, "read from " + out_robot_type[ii]);
    dhist->SetName(3 + 2 * ii, "write to " + out_robot_type[ii]);
  }
}
