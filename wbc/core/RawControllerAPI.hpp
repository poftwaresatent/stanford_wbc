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

#include <wbcrun/Factory.hpp>

class SAIVector;
class SAIMatrix;
struct timeval;

namespace wbc {
  
  
  class RobotControlModel;
  
  
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
  
  
  template<class RawControllerSubclass>
  class RawControllerFactory
    : public wbcrun::Factory<RawControllerSubclass, RawControllerAPI>
  {};
  
  
  class RawControllerFactoryRegistry
    : public wbcrun::FactoryRegistry<RawControllerAPI>
  {};
  
  
}

#endif // WBC_RAW_CONTROLLER_API_HPP
