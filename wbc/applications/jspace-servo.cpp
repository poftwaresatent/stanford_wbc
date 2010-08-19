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
   \file jspace-servo.cpp
   \author Roland Philippsen
*/

#include <wbc/bin/attributes.hpp>
#include <wbc/bin/options.hpp>
#include <wbc/core/Plugin.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/ServoInspector.hpp>
#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <jspace/RobotAPI.hpp>
#include <jspace/Model.hpp>
#include <jspace/controller_library.hpp>
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


static wbcnet::logger_t logger(wbcnet::get_logger("jspace-servo"));

static std::string const default_robot_type("fake");


namespace imp {

  
  class Inspector : public wbc::ServoInspector {
  public:
    virtual wbc::RobotControlModel * getRobotControlModel();
    virtual wbc::cop_data getCentersOfPressure();
    virtual SAIVector getCOM();
    virtual SAIVector getZMP();
  };
  
  
  struct options: public wbc::options {
    options()
      : robot_type(default_robot_type)
    {}
    
    virtual int unknown_option(int argc, char ** argv, int argnum) {
      if ((strlen(argv[argnum]) < 2) || (argv[argnum][0] != '-'))
	return -1;
      switch (argv[argnum][1]) {
      case 'H':
	cout << "XXXX reimplement -H option with plugin support\n";
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
      default:
	return -1;
      }
      return argnum;
    }
    
    virtual void specific_usage(std::ostream & os) const {
      os << "jspace-servo options:\n"
	 << "   -H             show help on available RobotAPI specs\n"
	 << "   -R  <spec>     RobotAPI specification (default " << default_robot_type << ")\n";
    }
    
    std::string robot_type;
  };
  
  
  class RobotAPI
    : public jspace::RobotAPI
  {
  public:
    RobotAPI(int ndof, wbc::RobotAPI * wbc_robot);
    
    virtual jspace::Status readState(jspace::State & state);
    virtual jspace::Status writeCommand(jspace::Vector const & command);
    virtual void shutdown();
    
    wbc::RobotAPI * wbc_robot_;
    timeval acquisition_time_;
    SAIVector joint_angles_;
    SAIVector joint_velocities_;
    SAIVector command_torques_;
  };
  
}


static imp::options opt;
static wbc::attributes * attr;
static wbc::RobotAPI * wbc_robot;
static imp::Inspector inspector;
static jspace::Model * jspace_model;
static jspace::Controller * jspace_controller;


static void handle(int signum)
{
  errx(SIGTERM == signum ? EXIT_SUCCESS : EXIT_FAILURE,
       "signal %d (%s)",
       signum, sys_siglist[signum]);
}


static void cleanup()
{
  if (wbc_robot) {
    wbc_robot->shutdown();
    delete wbc_robot;
  }
  delete jspace_model;
  delete jspace_controller;
  delete attr;
}


int main(int argc, char*argv[])
{
  if (0 != atexit(cleanup))
    err(EXIT_FAILURE, "atexit()");
  if (SIG_ERR == signal(SIGTERM, handle))
    err(EXIT_FAILURE, "signal(SIGTERM, ...)");
  if (SIG_ERR == signal(SIGPIPE, handle))
    err(EXIT_FAILURE, "signal(SIGPIPE, ...)");
  
  opt.parse(argc, argv);
  
  wbcnet::configure_logging();
  wbcnet::manual_logging_verbosity(opt.verbosity);
  
  try {
    attr = wbc::attributes::create(opt);
    wbc_robot = attr->extensions->robot_registry->parseCreate(opt.robot_type, &inspector);
  }
  catch (std::exception const & ee) {
    LOG_ERROR (logger, "EXCEPTION " << ee.what());
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
  
  LOG_INFO (logger, "creating and initializing model and state");
  jspace::tao_tree_info_s * kg_tree(attr->robmodel->branching()->createTreeInfo());
  jspace_model = new jspace::Model(/* transfers ownership */ kg_tree,
				   /* no Coriolis */ 0);
  int const ndof(jspace_model->getNDOF());
  jspace::State jspace_state;
  jspace_state.init(ndof, ndof, 0);
  imp::RobotAPI robot(ndof, wbc_robot);
  if ( ! robot.readState(jspace_state)) {
    LOG_ERROR (logger, "robot.readState() failed");
    errx(EXIT_FAILURE, "robot.readState() failed");
  }
  jspace_model->update(jspace_state);
  
  LOG_INFO (logger, "creating and initializing controller");
  // XXXX selectable at runtime...
  jspace::Vector initial_kp(ndof);
  jspace::Vector initial_kd(ndof);
  for (int ii(0); ii < ndof; ++ii) {
    initial_kp[ii] = 100;
    initial_kd[ii] = 20;
  }
  jspace_controller = new jspace::JointGoalController(jspace::COMP_GRAVITY |
						      jspace::COMP_MASS_INERTIA,
						      initial_kp, initial_kd);
  jspace::Status status;
  status = jspace_controller->init(*jspace_model);
  if ( ! status) {
    LOG_ERROR (logger, "jspace_controller->init() failed: " << status.errstr);
    errx(EXIT_FAILURE, "jspace_controller->init() failed: %s", status.errstr.c_str());
  }
  
  LOG_INFO (logger, "entering control loop");
  jspace::Vector tau(ndof);
  while (true) {
    
    status = jspace_controller->computeCommand(*jspace_model, tau);
    if ( ! status) {
      LOG_ERROR (logger, "jspace_controller->compute(tau) failed: " << status.errstr);
      errx(EXIT_FAILURE, "jspace_controller->compute(tau) failed: %s", status.errstr.c_str());
    }
    
    status = robot.writeCommand(tau);
    if ( ! status) {
      LOG_ERROR (logger, "robot.writeCommand() failed: " << status.errstr);
      errx(EXIT_FAILURE, "robot.writeCommand() failed: %s", status.errstr.c_str());
    }
    
    if ( ! robot.readState(jspace_state)) {
      LOG_ERROR (logger, "robot.readState() failed");
      errx(EXIT_FAILURE, "robot.readState() failed");
    }
    jspace_model->update(jspace_state);
    
  }
}


namespace imp {
  
  wbc::RobotControlModel * Inspector::
  getRobotControlModel()
  {
    if ( ! attr) {
      errx(EXIT_FAILURE, "imp::Inspector::getRobotControlModel() called before attr is available");
      errx(EXIT_FAILURE, "imp::Inspector::getRobotControlModel() called before attr is available");
    }
    if ( ! attr->robmodel) {
      errx(EXIT_FAILURE, "imp::Inspector::getRobotControlModel() called before robmodel is available");
      errx(EXIT_FAILURE, "imp::Inspector::getRobotControlModel() called before robmodel is available");
    }
    return attr->robmodel;
  }
  
  
  wbc::cop_data Inspector::
  getCentersOfPressure()
  {
    LOG_ERROR (logger, "imp::Inspector::getCentersOfPressure() should never be called");
    errx(EXIT_FAILURE, "imp::Inspector::getCentersOfPressure() should never be called");
    wbc::cop_data cop(0);
    return cop;
  }
  
  
  SAIVector Inspector::
  getCOM()
  {
    LOG_ERROR (logger, "imp::Inspector::getCOM() should never be called");
    errx(EXIT_FAILURE, "imp::Inspector::getCOM() should never be called");
    SAIVector com(0);
    return com;
  }
  
  
  SAIVector Inspector::
  getZMP()
  {
    LOG_ERROR (logger, "imp::Inspector::getZMP() should never be called");
    errx(EXIT_FAILURE, "imp::Inspector::getZMP() should never be called");
    SAIVector zmp(0);
    return zmp;
  }
  
  
  RobotAPI::
  RobotAPI(int ndof, wbc::RobotAPI * wbc_robot)
    : wbc_robot_(wbc_robot),
      joint_angles_(ndof),
      joint_velocities_(ndof),
      command_torques_(ndof)
  {
    acquisition_time_.tv_sec = 0;
    acquisition_time_.tv_usec = 0;
  }
  
  
  jspace::Status RobotAPI::
  readState(jspace::State & state)
  {
    jspace::Status st;
    if ( ! wbc_robot_->readSensors(joint_angles_, joint_velocities_, acquisition_time_, 0)) {
      st.ok = false;
      st.errstr = "wbc_robot_->readSensors() failed";
      return st;
    }
    state.time_sec_ = acquisition_time_.tv_sec;
    state.time_usec_ = acquisition_time_.tv_usec;
    state.position_.resize(joint_angles_.size());
    memcpy(&state.position_[0], joint_angles_.dataPtr(), joint_angles_.size() * sizeof(double));
    state.velocity_.resize(joint_velocities_.size());
    memcpy(&state.velocity_[0], joint_velocities_.dataPtr(), joint_velocities_.size() * sizeof(double));
    state.force_.resize(0);
    return st;
  }
  
  
  jspace::Status RobotAPI::
  writeCommand(jspace::Vector const & command)
  {
    command_torques_.setSize(command.size());
    memcpy(command_torques_.dataPtr(),
	   &const_cast<jspace::Vector&>(command).coeffRef(0),
	   command.size() * sizeof(double));
    jspace::Status st;
    if ( ! wbc_robot_->writeCommand(command_torques_)) {
      st.ok = false;
      st.errstr = "wbc_robot_->writeCommand() failed";
      return st;
    }
    return st;
  }
  
  
  void RobotAPI::
  shutdown()
  {
    wbc_robot_->shutdown();
  }
  
}
