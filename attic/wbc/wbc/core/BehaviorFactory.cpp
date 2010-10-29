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
 * \file BehaviorFactory.cpp
 * \author Roland Philippsen
 */

#include <wbc/core/BehaviorFactory.hpp>
#include <wbc/motion/FloatBehavior.hpp>
#include <wbc/motion/PostureBehavior.hpp>
#include <sstream>

using namespace std;


namespace wbc {
  
  BehaviorFactoryRegistry::
  BehaviorFactoryRegistry(RobotControlModel * robmodel)
    : m_robmodel(robmodel)
  {
    Add("FloatBehavior",   new BehaviorFactory<FloatBehavior>());
    Add("PostureBehavior", new BehaviorFactory<PostureBehavior>());
  }
  
  
  BehaviorDescription * BehaviorFactoryRegistry::
  Create(std::string const & name) const throw(std::runtime_error)
  {
    BehaviorDescription * behavior(Get(name)->create());
    cout << "BehaviorFactoryRegistry::Create(): initializing " << name << "\n";
    behavior->robotControlModel(m_robmodel);
    return behavior;
  }

}
