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
   \file RawControllerAPI.hpp
   \author Roland Philippsen
*/

#ifndef WBC_RAW_CONTROLLER_API_HPP
#define WBC_RAW_CONTROLLER_API_HPP

#include <wbcnet/Factory.hpp>

class SAIVector;
class SAIMatrix;
struct timeval;

namespace wbc {
  
  
  class RobotControlModel;
  
  
  /**
     Abstract interface for "raw" controllers. All they get from the
     framework is a RobotControlModel and the connection with a
     RobotAPI. Everything else has to be provided by the
     implementer.
     
     You can add a RawControllerAPI subclass to your plugin, and use
     the \c rawservo executable to run it. The \c rawservo will take
     care of plugin loading, network configuration, instantiation of a
     robot API from the command line, and robot description file
     parsing.
     
     See the plugins/fake/plugin.cpp for an example of registering a
     RawControllerAPI subclass with the WBC runtime extension
     infrastructure.
  */
  class RawControllerAPI
  {
  public:
    virtual ~RawControllerAPI() {}
    
    virtual bool computeTorques(RobotControlModel const & robot_model,
				SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				SAIVector & command_torques) = 0;
  };
  
  
  /**
     Generic factory for RawControllerAPI subclasses that are default
     constructible. If you need constructor arguments for your raw
     controller, you are probably better off copy-paste-adapting the
     applications/rawservo.cpp file.
   */
  template<class RawControllerSubclass>
  class RawControllerFactory
    : public wbcnet::Factory<RawControllerSubclass, RawControllerAPI>
  {};
  
  
  /**
     A registry of RawControllerFactory instances. This is used by the
     plugin mechanism to collect all available raw controllers.
  */
  class RawControllerFactoryRegistry
    : public wbcnet::FactoryRegistry<RawControllerAPI>
  {};
  
  
}

#endif // WBC_RAW_CONTROLLER_API_HPP
