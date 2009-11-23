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

//=========================================================================
/*!
  \file       FloatBehavior.hpp
  \author     Luis Sentis
*/
//=========================================================================
 
#ifndef WBC_FLOAT_BEHAVIOR_HPP
#define WBC_FLOAT_BEHAVIOR_HPP

#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/TaskSet.hpp>
#include <wbc/motion/FrictionPosture.hpp>
#include <wbc/core/RobotControlModel.hpp>

namespace wbc {

  class FloatBehavior
    : public BehaviorDescription
  {
  public:

    FloatBehavior();
    ~FloatBehavior() {}

    virtual TaskSet* activeTaskSet() { return activeTaskSet_; }
    virtual void onUpdate();

  protected:
    virtual void loadMovementPrimitives( RobotControlModel* ) throw(std::runtime_error); 
  
  private:
    // General attributes
    TaskSet taskSet_;
    TaskSet* activeTaskSet_;
    FrictionPosture friction_posture_;
    unsigned int currentState_;
  };

}

#endif
