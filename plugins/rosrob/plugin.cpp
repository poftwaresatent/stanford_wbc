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
   \file plugins/rosrob/plugin.cpp
   \author Roland Philippsen
*/

#include "Robot.hpp"
#include <wbc/core/Plugin.hpp>

using namespace std;

namespace wbc_rosrob_plugin {
  
  class Plugin: public wbc::Plugin {
  public:
    virtual void Init(wbc::Extensions & extensions) throw(std::runtime_error)
    {
      extensions.AddRobot("ros", new Factory());
    }
  };

}

wbcnet::Module * wbcnet_create_module()
{
  return new wbc_rosrob_plugin::Plugin();
}
