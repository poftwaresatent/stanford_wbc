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
   \file wbcservo.cpp
   \author Roland Philippsen
*/

#include <wbc/bin/attributes.hpp>
#include <wbc/bin/options.hpp>
#include <wbc/bin/ModelProcess.hpp>
#include <wbc/core/Plugin.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/ServoInspector.hpp>
#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/core/Contact.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbcnet/misc/DelayHistogram.hpp>
#include <wbcnet/log.hpp>
#include <wbc/bin/ServoProcess.hpp>
#include <wbc/bin/ServoModelProcess.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/ServoBehaviorsAPI.hpp>
#include <wbc/core/TaskModelFactory.hpp>
#include <wbc/core/TaskModelBase.hpp>
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

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

static double const default_servo_rate(500);
static string const default_robot_type("fake");
	
	
namespace servo {
  
  class Inspector : public wbc::ServoInspector {
  public:
    virtual RobotControlModel * getRobotControlModel();
    virtual wbc::cop_data getCentersOfPressure();
    virtual SAIVector getCOM();
    virtual SAIVector getZMP();
  };
  
  
  struct options: public wbc::options {
    options()
      : servo_rate(default_servo_rate),
	robot_type(default_robot_type),
	start_with_torques(false),
	enable_multirate(true),
	pskip(0) {}
    
    virtual int unknown_option(int argc, char ** argv, int argnum) {
      if ((strlen(argv[argnum]) < 2) || (argv[argnum][0] != '-'))
	return -1;
      switch (argv[argnum][1]) {
      case 'H':
#warning 'XXXX reimplement -H option with plugin support'
	cout << "XXXX reimplement -H option with plugin support\n";
	////	robot_reg.dumpAll("  ", cout);
	exit(EXIT_SUCCESS);
      case 'm':
	enable_multirate = false;
	break;
      case 'R':
	++argnum;
	if (argnum >= argc) {
	  cerr << argv[0] << ": -R requires robot spec (see -H for a list of available robots)\n";
	  usage(cerr);
	  return -1;
	}
	robot_type = argv[argnum];
	break;
      case 'P':			// upper-case
	start_with_torques = true;
	break;
      case 'r':
	++argnum;
	if (argnum >= argc) {
	  cerr << argv[0] << ": -r requires a rate argument\n";
	  return -1;
	}
	{
	  istringstream is(argv[argnum]);
	  if ( ! (is >> servo_rate)) {
	    cerr << argv[0] << ": error reading servo_rate argument '" << argv[argnum] << "'\n";
	    return -1;
	  }
	}
	break;
      case 'p':			// lower-case
	++argnum;
	if (argnum >= argc) {
	  cerr << argv[0] << ": -p requires a skip argument\n";
	  return -1;
	}
	{
	  istringstream is(argv[argnum]);
	  if ( ! (is >> pskip)) {
	    cerr << argv[0] << ": error reading skip argument '" << argv[argnum] << "'\n";
	    return -1;
	  }
	}
	break;
      default:
	return -1;
      }
      return argnum;
    }
    
    virtual void specific_usage(std::ostream & os) const {
      os << "servo options:\n"
	 << "   -H             show help on available RobotAPI specs\n"
	 << "   -m             run model inside servo (NO separate model process)\n"
	 << "   -P             start by first sending some zero torques\n"
	 << "   -R  <spec>     RobotAPI specification (default " << default_robot_type << ")\n"
	 << "   -r  <rate Hz>  desired update rate (default " << default_servo_rate << ")\n"
	 << "   -p  <skip>     print q, tau, and delay histogram every skip loops (0 to disable)\n";
    }
    
    double servo_rate;
    string robot_type;
    bool start_with_torques;
    bool enable_multirate;
    int pskip;
  };
  
}


static servo::options opt;
static attributes * attr;
static ServoImplementation * imp(0);
static wbc::Process * proc(0);
static RobotAPI * robotAPI;
static wbcnet::DelayHistogram * dhist;
static ServoBehaviorsAPI * servoBehaviors;
static servo::Inspector inspector;
static std::vector<wbc::TaskModelBase*> task_model_pool;


static void handle(int signum)
{
  errx(SIGTERM == signum ? EXIT_SUCCESS : EXIT_FAILURE,
       "signal %d (%s)",
       signum, sys_siglist[signum]);
}


static void cleanup()
{
  delete proc;
  delete imp;
  delete servoBehaviors;
  delete dhist;
  delete robotAPI;
  for (size_t ii(0); ii < task_model_pool.size(); ++ii)
    delete task_model_pool[ii];
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
  if (opt.servo_rate <= 0)
    errx(EXIT_FAILURE, "invalid servo_rate %g", opt.servo_rate);
  if (opt.pskip > 0) {
    double const ms_servo_rate(1e3 / opt.servo_rate);
    dhist = new wbcnet::DelayHistogram(1, 30, 0.5 * ms_servo_rate, 3.5 * ms_servo_rate, opt.pskip);
    dhist->SetName(0, "read sensor loop");
  }
  
  wbcnet::configure_logging();
  wbcnet::manual_logging_verbosity(opt.verbosity);
  
  try {
    attr = attributes::create(opt);
    static wbcnet::endian_mode_t const endian_mode(wbcnet::ENDIAN_DETECT);
    for (size_t ii(0); ii < 2; ++ii)
      task_model_pool.push_back(attr->extensions->task_model_registry->Create(opt.motion_type,
									     attr->robmodel->branching(),
									     endian_mode));
    robotAPI = attr->extensions->robot_registry->parseCreate(opt.robot_type, &inspector);
    servoBehaviors = attr->extensions->servo_behaviors_registry->Create(opt.motion_type);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
  
  warnx("initializing wbc::ServoProcess");
  size_t const ndof(attr->robmodel->branching()->numJoints());
  size_t const ndof_actuated(attr->robmodel->branching()->numActuatedJoints());
  size_t const nvel(ndof); // one day maybe we will actually have spherical joints... then these will differ
  size_t contact_nrows, contact_ncols;
  attr->robmodel->getForceDimension(contact_nrows, contact_ncols);
  imp = new ServoImplementation(ndof,
				ndof_actuated,
				nvel,
				contact_nrows,
				contact_ncols,
				attr->robmodel,
				servoBehaviors,
				attr->behavior,
				task_model_pool,
				robotAPI,
				dhist,
				opt.pskip);
  
  try {
    if (opt.enable_multirate) {
      warnx("multirate servo / model update ENABLED");
      wbc::ServoProcess *
	rfct(new wbc::ServoProcess());
      proc = rfct;
      rfct->Init(imp, false, *attr->netcfg, ndof, nvel, contact_nrows, contact_ncols);
    }
    else {
      warnx("single-process servo and model (multirate DISABLED)");
      wbc::ServoModelProcess *
	rfct(new wbc::ServoModelProcess());
      proc = rfct;
      ModelImplementation *
	model_imp(new ModelImplementation(task_model_pool[0],
					  attr->robmodel,
					  attr->behavior));
      rfct->Init(imp, false, model_imp, true, *attr->netcfg, ndof, nvel, contact_nrows, contact_ncols);
    }
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
  
  if (opt.start_with_torques) {
    warnx("starting by sending zero torques");
    if ( ! imp->NullTorqueCommand())
      errx(EXIT_FAILURE, "imp->NullTorqueCommand() failed");
  }
  
  warnx("entering main loop");
  while (true) {
    proc->Step();
////    imp->Record();
    robotAPI->updateDynamics(attr->robmodel);
    if (dhist)
      dhist->CheckDumpTable(stdout);
  }
}


namespace servo {
  
  RobotControlModel * Inspector::
  getRobotControlModel()
  {
    if ( ! attr)
      errx(EXIT_FAILURE, "servo::Inspector::getRobotControlModel() called before attr is available");
    if ( ! attr->robmodel)
      errx(EXIT_FAILURE, "servo::Inspector::getRobotControlModel() called before robmodel is available");
    return attr->robmodel;
  }
  
  
  wbc::cop_data Inspector::
  getCentersOfPressure()
  {
    if ( ! imp)
      errx(EXIT_FAILURE, "servo::Inspector::getCentersOfPressure() called before imp is available");
    Contact::cop_map_t const copMap(attr->robmodel->contact()->copMap());
    wbc::cop_data cop(copMap.size());
    for (Contact::cop_map_t::const_iterator ic(copMap.begin()); ic != copMap.end(); ++ic) {
      if (attr->robmodel->contact()->linkIsInContact(ic->first)) {
	cop.linkID.push_back(ic->first);
	cop.cop.push_back(ic->second);
      }
    }
    return cop;
  }
  
  
  SAIVector Inspector::
  getCOM()
  {
    if ( ! imp)
      errx(EXIT_FAILURE, "servo::Inspector::getCOM() called before imp is available");
    SAIVector com(attr->robmodel->kinematics()->COM());
    return com;
  }
  
  
  SAIVector Inspector::
  getZMP()
  {
    if ( ! imp)
      errx(EXIT_FAILURE, "servo::Inspector::getZMP() called before imp is available");
    SAIVector zmp;
    attr->robmodel->contact()->zmp(zmp);
    return zmp;
  }
  
}
