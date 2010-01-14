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
 * \file BehaviorFactory.hpp
 * \author Roland Philippsen
 */

#ifndef WBC_BEHAVIOR_FACTORY_HPP
#define WBC_BEHAVIOR_FACTORY_HPP

#include <wbcnet/Registry.hpp>
#include <stdexcept>
#include <string>

namespace wbc {
  
  class RobotControlModel;
  class BehaviorDescription;
  
  class BehaviorFactoryAPI {  
  public:
    virtual ~BehaviorFactoryAPI() {}
    virtual BehaviorDescription * create() = 0;
  };
  
  /**
     Should be enough for most if not all cases.
  */
  template<typename behavior_t>
  class BehaviorFactory
    : public BehaviorFactoryAPI {  
  public:
    virtual BehaviorDescription * create() { return new behavior_t(); }
  };
  
  
  typedef wbcnet::Registry<BehaviorFactoryAPI *,
			   wbcnet::registry_trait_delete<BehaviorFactoryAPI *> >
  BehaviorFactoryRegistrySuper;
  
  /**
     \note Use the superclass' Add() to register BehaviorFactory
     instances with this registry.
  */
  class BehaviorFactoryRegistry
    : public BehaviorFactoryRegistrySuper
  {
  public:
    /**
       Also registers some standard behavior factories.
    */
    explicit BehaviorFactoryRegistry(RobotControlModel * robmodel);
    
    BehaviorDescription * Create(std::string const & name) const throw(std::runtime_error);
    
  protected:
    mutable RobotControlModel * m_robmodel;
  };

}

#endif // BEHAVIOR_FACTORY_HPP
