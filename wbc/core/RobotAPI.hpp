/*
 * Stanford_WBC_Extension -- plugin library for stanford-wbc.sourceforge.net
 *
 * Copyright (C) 2008, 2009 Stanford University
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

#ifndef WBC_ROBOT_API_HPP
#define WBC_ROBOT_API_HPP

#ifndef WIN32
#include <sys/types.h>
#else
#include "extras.h"
#endif

#include <iosfwd>

class SAIVector;
class SAIMatrix;
struct timeval;


namespace wbc {

  class RobotControlModel;
  
  
  /**
     This allows us to easily switch between the real robot, the
     connection to SAI simulation, and other possibilities. The various
     subclasses take care of the necessary differences. Use
     RobotFactory::parseCreate() to instantiate RobotAPI subclasses
     without making your code dependent on what's out there.
  */
  class RobotAPI
  {
  public:
    virtual ~RobotAPI() {}
    
    /**
       Interface for reading the current (generalized) state of the
       robot. The minimum required information are the joint angles
       and velocities, as well as the acquisition time of this state
       information. Optionally, the caller can ask for contact force
       data by passing a non-null \c opt_forces pointer. In case the
       robot you are implementing does not have force sensors, and the
       \c opt_forces pointer is non-null, you should explicitly set
       the given SAIMatrix to zero size.
       
       \arg \c jointAngles out-parameter that must be filled-in with
       the current (generalized) joint coordinates.
       
       \arg \c jointVelocities out-parameter that must be filled with
       the derivatives of the current (generalized) joint coordinates.
       
       \arg \c acquisition_time out-parameter that must be set to the
       instant of time which refers to the "current" state. This is
       useful when running in simulation: it makes it possible to let
       the servo run according to simulated time, instead of wallclock
       time.
       
       \arg \c opt_forces optional out-parameter: a 6-by-NDOF (generalized DOFs) matrix
       where each column contains a contact's 3 force components
       stacked on top of the 3 torque components. The column index
       indicates which joint the corresponding forces and torques are
       applied to (you are responsible for expressing the forces on
       the link with respect to the link's joint --- and also to fuse
       all given force sensors into one per joint in case you have
       multiple sensors). Pad the matrix with zeros for joints that do
       not have this data. Set the matrix to zero size in case your
       robot does not support contact forces at all.
       
       \return true unless an error occurred. Note that the servo loop
       will exit with an error in case your implementation of
       readSensors() returns false.
     */
    virtual bool
    readSensors(SAIVector & jointAngles,
		SAIVector & jointVelocities,
		timeval & acquisition_time,
		SAIMatrix * opt_forces) = 0;
  
    
    /**
       Interface used by the servo to send torque commands to the
       robot. The output of the whole-body controller is simply a
       vector of generalized torques, to be applied to the joints of
       the robot.
    */
    virtual bool
    writeCommand(SAIVector const & command) = 0;
    
    virtual void shutdown() const = 0;
    
    /**
       Hook for debugging your RobotAPI.
       \note Default implementation does nothing.
    */
    virtual void dumpStats(char const * prefix, std::ostream & os) const {}
    
    /** \todo XXXX virtual because of quick rfct for RobotSim plugin,
	should be solved more properly */
    virtual bool updateDynamics(RobotControlModel * robmodel) { return false; }
  };
  
  
  /**
     An API of a robot that can pretend to be a source of servo
     commands and a sink of robot state.  Some RobotAPI subclasses are
     actually just proxies that sit on top of some messaging protocol,
     and they can be used in either direction. So letting them
     subclass this specialized form allows us to write bridges between
     different robot implementations.
   */
  class BidirectionalRobotAPI
    : public RobotAPI
  {
  public:
    /**
       A sink for the stuff returned as out-parameters by RobotAPI::readSensors().
    */
    virtual bool writeSensors(SAIVector const & jointAngles, SAIVector const & jointVelocities,
			      SAIMatrix const * opt_forces) = 0;
    
    /**
       A source for the stuff taken as parameter by
       RobotAPI::writeCommand().
    */
    virtual bool readCommand(SAIVector & command) = 0;
  };
  
}

#endif // WBC_ROBOT_API_HPP
