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
   \file MobileManipulatorServoBehaviors.hpp
   \author Roland Philippsen and Luis Sentis
*/

#ifndef WBC_MOBILE_MANIPULATOR_SERVO_BEHAVIORS_HPP
#define WBC_MOBILE_MANIPULATOR_SERVO_BEHAVIORS_HPP

#include <wbc/core/ServoBehaviorsAPI.hpp>

namespace wbc {
  
  class MobileManipulatorServoBehaviors
    : public ServoBehaviorsAPI
  {
  public:
    /** \todo \c behavior and \c robmodel should be const pointers,
	but that requires some wider refactorings first. */
    virtual bool updateTorques( /** in  */ BehaviorDescription * behavior,
				/** in  */ RobotControlModel * robmodel,
				/** in  */ wbcnet::TaskModelAPI const * taskModel,
				/** out */ SAIVector & generalizedTorques);
    
    virtual void record( Time const & now );
  };
  
}

#endif // WBC_MOBILE_MANIPULATOR_SERVO_BEHAVIORS_HPP
