/*
 * ROS support for Stanford-WBC http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file wbc/ros/BehaviorLibrary.hpp
   \author Roland Philippsen
*/

#ifndef WBCROS_BEHAVIOR_LIBRARY_HPP
#define WBCROS_BEHAVIOR_LIBRARY_HPP

#include <wbc/ros/Model.hpp>

namespace wbc {
  class BehaviorDescription;
  class BehaviorFactoryRegistry;
}

namespace wbcros {
  
  class BehaviorLibrary
  {
  public:
    std::string behaviors_param_name_;
    std::vector<wbc::BehaviorDescription*> behavior_;
    
    BehaviorLibrary(std::string const & param_prefix);
    ~BehaviorLibrary();
    
    void initDummy(Model & model);
    
    void initFromParam(Model & model, ros::NodeHandle &nn,
		       wbc::BehaviorFactoryRegistry const & breg) throw(std::runtime_error);
  };
  
}

#endif // WBCROS_BEHAVIOR_LIBRARY_HPP
