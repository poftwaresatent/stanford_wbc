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
   \file builtin.cpp
   \author Roland Philippsen
*/

#include <wbc/core/Plugin.hpp>
#include <wbc/core/MobileManipulatorTaskModel.hpp>
#include <wbc/core/MobileManipulatorServoBehaviors.hpp>
#include <wbcnet/log.hpp>

#ifdef DISABLE_PLUGINS

// Needed at link time, don't care where from though. See
// example_wbc_add_builtin_plugins.cpp for an example.
void wbc_add_builtin_plugins(wbc::Extensions * extensions) throw(std::runtime_error);

#endif // DISABLE_PLUGINS

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

namespace wbc {
  
  Extensions * load_extensions(RobotControlModel * robmodel) throw(std::exception)
  {
    Extensions * extensions(new Extensions(robmodel));
    try {
      LOG_INFO (logger, "wbc::load_extensions(): adding builtin model factories");
      extensions->AddTaskModel("mobile", new MobileManipulatorTaskModelFactory());
      extensions->AddServoBehaviors("mobile", new ServoBehaviorsFactory<MobileManipulatorServoBehaviors>());
    }
    catch (std::exception const & ee) {
      LOG_ERROR (logger,
		 "wbc::load_extensions(): EXCEPTION during registering of builtin task model factories: "
		 << ee.what());
      delete extensions;
      throw ee;
    }
    
#ifdef DISABLE_PLUGINS
    
    LOG_INFO (logger, "wbc::load_extensions(): dynamic loading of plugins disabled, using static extensions");
    wbc_add_builtin_plugins(extensions);
    
#else // DISABLE_PLUGINS

    // XXXX registry should be deleted after use (when the program exits)
    PluginRegistry * plugin_reg(new PluginRegistry(*extensions));
    LOG_INFO (logger, "wbc::load_extensions(): loading plugins...");
    size_t const nplugins(plugin_reg->SearchAndLoadPlugins());
    LOG_INFO (logger, "wbc::load_extensions(): loaded " << nplugins << " plugins");

#endif // DISABLE_PLUGINS
    
    return extensions;
  }
  
}

