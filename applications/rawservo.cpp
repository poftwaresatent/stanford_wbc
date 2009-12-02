/*
 * Copyright (c) 2009 Stanford University
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
   \file rawservo.cpp
   \author Roland Philippsen
*/

#include <wbc/bin/attributes.hpp>
#include <wbc/bin/options.hpp>
#include <wbc/core/Plugin.hpp>
#include <wbc/core/RawControllerAPI.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/ServoInspector.hpp>
#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbcnet/endian_mode.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <sstream>
#include <cstring>
#include <err.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>


using namespace std;
using namespace wbc;

static wbcnet::logger_t logger(wbcnet::get_logger("rawservo"));

static string const default_robot_type("fake");
static string const default_controller_type("fake");


namespace rawservo {
  
  class Inspector : public wbc::ServoInspector {
  public:
    virtual RobotControlModel * getRobotControlModel();
    virtual wbc::cop_data getCentersOfPressure();
    virtual SAIVector getCOM();
    virtual SAIVector getZMP();
  };
  
  
  struct options: public wbc::options {
    options()
      : robot_type(default_robot_type),
	controller_type(default_controller_type)
    {}
    
    virtual int unknown_option(int argc, char ** argv, int argnum) {
      if ((strlen(argv[argnum]) < 2) || (argv[argnum][0] != '-'))
	return -1;
      switch (argv[argnum][1]) {
      case 'H':
#warning 'XXXX reimplement -H option with plugin support'
	cout << "XXXX reimplement -H option with plugin support\n";
	////	robot_reg.dumpAll("  ", cout);
	////	controller_reg.dumpAll("  ", cout);
	exit(EXIT_SUCCESS);
      case 'R':
	++argnum;
	if (argnum >= argc) {
	  cerr << argv[0] << ": -R requires robot spec (see -H for a list of available robots)\n";
	  usage(cerr);
	  return -1;
	}
	robot_type = argv[argnum];
	break;
      case 'C':
	++argnum;
	if (argnum >= argc) {
	  cerr << argv[0] << ": -C requires controller spec (see -H for a list of available controllers)\n";
	  usage(cerr);
	  return -1;
	}
	controller_type = argv[argnum];
	break;
      default:
	return -1;
      }
      return argnum;
    }
    
    virtual void specific_usage(std::ostream & os) const {
      os << "rawservo options:\n"
	 << "   -H             show help on available RobotAPI specs\n"
	 << "   -R  <spec>     RobotAPI specification (default " << default_robot_type << ")\n"
	 << "   -C  <spec>     RawControllerAPI specification (default " << default_controller_type << ")\n";
    }
    
    string robot_type;
    string controller_type;
  };
  
}


static rawservo::options opt;
static attributes * attr;
static RobotAPI * robotAPI;
static rawservo::Inspector inspector;
static RawControllerAPI * controller;


static void handle(int signum)
{
  errx(SIGTERM == signum ? EXIT_SUCCESS : EXIT_FAILURE,
       "signal %d (%s)",
       signum, sys_siglist[signum]);
}


static void cleanup()
{
  delete controller;
  if (robotAPI) {
    robotAPI->shutdown();
    delete robotAPI;
  }
  delete attr;
}


int main(int argc, char*argv[])
{
  if ((0 != cleanup) && (0 != atexit(cleanup)))
    err(EXIT_FAILURE, "atexit()");
  if (SIG_ERR == signal(SIGTERM, handle))
    err(EXIT_FAILURE, "signal(SIGTERM, ...)");
  if (SIG_ERR == signal(SIGPIPE, handle))
    err(EXIT_FAILURE, "signal(SIGPIPE, ...)");
  
  opt.parse(argc, argv);
  
  wbcnet::configure_logging();
  wbcnet::manual_logging_verbosity(opt.verbosity);
  
  try {
    attr = attributes::create(opt);
    static wbcnet::endian_mode_t const endian_mode(wbcnet::ENDIAN_DETECT);
    robotAPI = attr->extensions->robot_registry->parseCreate(opt.robot_type, &inspector);
    controller = attr->extensions->raw_controller_registry->Create(opt.controller_type);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
  
  SAIVector joint_angles(attr->robmodel->branching()->numJoints());
  SAIVector joint_velocities(attr->robmodel->branching()->numJoints());
  timeval acquisition_time;
  acquisition_time.tv_sec = 0;
  acquisition_time.tv_usec = 0;
  size_t nrows, ncols;
  attr->robmodel->getForceDimension(nrows, ncols);
  SAIMatrix contact_forces(nrows, ncols);
  SAIVector command_torques(attr->robmodel->branching()->numJoints());
  
  while (true) {
    
    if ( ! robotAPI->readSensors(joint_angles, joint_velocities, acquisition_time, &contact_forces)) {
      errx(EXIT_FAILURE, "robotAPI->readSensors() failed");
    }
    
    if ( ! controller->computeTorques(*attr->robmodel,
				      joint_angles,
				      joint_velocities,
				      acquisition_time,
				      contact_forces,
				      command_torques)) {
      errx(EXIT_FAILURE, "controller->computeTorques() failed");
    }
    
    if ( ! robotAPI->writeCommand(command_torques)) {
      errx(EXIT_FAILURE, "robotAPI->writeCommand() failed");
    }
    
    robotAPI->updateDynamics(attr->robmodel);
  }
}


namespace rawservo {
  
  RobotControlModel * Inspector::
  getRobotControlModel()
  {
    if ( ! attr)
      errx(EXIT_FAILURE, "rawservo::Inspector::getRobotControlModel() called before attr is available");
    if ( ! attr->robmodel)
      errx(EXIT_FAILURE, "rawservo::Inspector::getRobotControlModel() called before robmodel is available");
    return attr->robmodel;
  }
  
  
  wbc::cop_data Inspector::
  getCentersOfPressure()
  {
    errx(EXIT_FAILURE, "rawservo::Inspector::getCentersOfPressure() should never be called");
    wbc::cop_data cop(0);
    return cop;
  }
  
  
  SAIVector Inspector::
  getCOM()
  {
    errx(EXIT_FAILURE, "rawservo::Inspector::getCOM() should never be called");
    SAIVector com(0);
    return com;
  }
  
  
  SAIVector Inspector::
  getZMP()
  {
    errx(EXIT_FAILURE, "servo::Inspector::getZMP() should never be called");
    SAIVector zmp(0);
    return zmp;
  }
  
}
