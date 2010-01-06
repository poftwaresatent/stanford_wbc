/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
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
   \file plugins/robotlog/plugin.cpp
   \author Roland Philippsen
*/

#include "readlog.hpp"
#include "writelog.hpp"
#include <wbc/core/Plugin.hpp>

namespace {
  
  class Plugin: public wbc::Plugin {
  public:
    virtual void Init(wbc::Extensions & extensions) throw(std::runtime_error)
    {
      extensions.AddRobot("rlog", new RLogFactory());
      extensions.AddRobot("wlog", new WLogFactory(extensions));
    }
  };
  
}

wbcnet::Module * wbcnet_create_module()
{
  return new Plugin();
}
