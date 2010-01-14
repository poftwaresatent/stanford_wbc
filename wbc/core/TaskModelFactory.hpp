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
   \file TaskModelFactory.hpp Utility for creating wbcnet::TaskModelAPI subclasses from plugins.
   \author Roland Philippsen
 */

#ifndef WBC_TASK_MODEL_FACTORY_HPP
#define WBC_TASK_MODEL_FACTORY_HPP

#include <wbcnet/endian_mode.hpp>
#include <wbcnet/Registry.hpp>
#include <string>
#include <iosfwd>

namespace wbc {
  
  class BranchingRepresentation;
  class TaskModelBase;
  
  
  class TaskModelFactoryAPI
  {
  public:
    virtual ~TaskModelFactoryAPI() {}
    
    virtual TaskModelBase * Create(BranchingRepresentation * branching,
				   wbcnet::endian_mode_t endian_mode) = 0;
  };
  
  
  typedef wbcnet::Registry<TaskModelFactoryAPI *,
			   wbcnet::registry_trait_delete<TaskModelFactoryAPI *> >
  TaskModelFactoryRegistrySuper;
  
  /**
     \note Use the superclass' Add() to register TaskModelFactory
     instances with this registry.
  */
  class TaskModelFactoryRegistry
    : public TaskModelFactoryRegistrySuper
  {
  public:
    inline TaskModelBase * Create(std::string const & name,
				  BranchingRepresentation * branching,
				  wbcnet::endian_mode_t endian_mode) const throw(std::runtime_error)
    { return Get(name)->Create(branching, endian_mode); }
  };
  
}

#endif // WBC_TASK_MODEL_FACTORY_HPP
