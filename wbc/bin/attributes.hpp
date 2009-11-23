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
   \file attributes.hpp
   \author Roland Philippsen
*/

#ifndef WBC_ATTRIBUTES_HPP
#define WBC_ATTRIBUTES_HPP

#include <string>
#include <vector>
#include <stdexcept>

namespace wbcnet {
  class NetConfig;
}

namespace wbc {
  
  struct options;
  class RobotControlModel;
  class BehaviorDescription;
  class Extensions;
  class PluginRegistry;
  
  
  struct attributes
  {
    attributes();
    ~attributes();
    
    static attributes * create(options const & opt) throw(std::runtime_error);
    
    wbc::Extensions * extensions;
    wbcnet::NetConfig const * netcfg;
    RobotControlModel * robmodel;
    std::vector<wbc::BehaviorDescription*> behavior;
  };
  
}

#endif // WBC_ATTRIBUTES_HPP
