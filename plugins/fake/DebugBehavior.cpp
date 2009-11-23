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
   \author Roland Philippsen
*/

#include "DebugBehavior.hpp"
#include <wbc/core/TaskSet.hpp>
#include <wbcrun/service.hpp>


DebugBehavior::
DebugBehavior()
  : BehaviorDescription("DebugBehavior"),
    friction_posture_("friction_posture")
{
}


TaskSet * DebugBehavior::
activeTaskSet()
{
  return &taskSet_;
}


void DebugBehavior::
onUpdate()
{
  friction_posture_.onUpdate();
}


void DebugBehavior::
loadMovementPrimitives( RobotControlModel* robmodel )
  throw(std::runtime_error)
{
  friction_posture_.robotControlModel( robmodel );
  taskSet_.addTask( &friction_posture_ );
  registerTaskSet(&taskSet_);
  friction_posture_.diffGain(11.0);
}


int32_t DebugBehavior::
handleKey(int32_t keycode)
{
  std::cout << "DebugBehavior::handleKey(" << keycode << ")\n";
  return wbcrun::srv::SUCCESS;
}
