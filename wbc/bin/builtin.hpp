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
   \file builtin.hpp
   \author Roland Philippsen
*/

#ifndef WBC_BUILTIN_HPP
#define WBC_BUILTIN_HPP

namespace wbc {
  
  class Extensions;
  
  /**
     Create an Extensions instance, populate it with some builtin
     functionality, and use PluginRegistry::SearchAndLoadPlugins() to
     populate it further from loadable modules.
     
     \return A newly created and populated Extensions instance, which
     the caller has to delete when they are done with it. In case of
     an error, an exception is thrown.
  */
  Extensions * load_extensions(RobotControlModel * robmodel) throw(std::exception);
  
}

#endif // WBC_BUILTIN_HPP


