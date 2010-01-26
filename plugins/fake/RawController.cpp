/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file RawController.cpp
   \author Roland Philippsen
*/

#include "RawController.hpp"
#include <wbcnet/log.hpp>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <sstream>
#include <sys/time.h>
#include <math.h>

using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("fake"));

namespace wbc_fake_plugin {
  
  bool RawController::
  computeTorques(wbc::RobotControlModel const & robot_model,
		 SAIVector const & joint_angles,
		 SAIVector const & joint_velocities,
		 timeval const & acquisition_time,
		 SAIMatrix const & contact_forces,
		 SAIVector & command_torques)
  {
    double amplitude(1e-3 * (acquisition_time.tv_usec % 1000));
    double phase(M_PI * (acquisition_time.tv_sec % 36) / 18.0);
    for (int ii(0); ii < command_torques.size(); ++ii) {
      command_torques[ii] = amplitude * cos(phase + M_PI * ii / 18.0);
    }
    
    if (logger->isInfoEnabled()) {
      ostringstream msg;
      msg << "fake::RawController::computeTorques():\n"
	  << "  acquisition_time: " << acquisition_time.tv_sec
	  << "s " << acquisition_time.tv_usec << "us\n";
      joint_angles.prettyPrint(msg,     "  joint_angles",     "    ");
      joint_velocities.prettyPrint(msg, "  joint_velocities", "    ");
      contact_forces.prettyPrint(msg,   "  contact_forces",   "    ");
      command_torques.prettyPrint(msg,  "  command_torques",  "    ");
      LOG_INFO (logger, msg.str());
    }
    
    return true;
  }
  
}
