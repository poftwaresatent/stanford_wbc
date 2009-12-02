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
   \author Roland Philippsen
*/

#ifndef FAKE_RAW_CONTROLLER_HPP
#define FAKE_RAW_CONTROLLER_HPP

#include <wbc/core/RawControllerAPI.hpp>

namespace fake {
  
  class RawController
    : public wbc::RawControllerAPI
  {
  public:
    virtual bool computeTorques(wbc::RobotControlModel const & robot_model,
				SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				SAIVector & command_torques);
  };
  
}

#endif // FAKE_RAW_CONTROLLER_HPP
