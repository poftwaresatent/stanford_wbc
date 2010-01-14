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
  \file PostureBehavior.hpp
  \author Luis Sentis and Roland Philippsen
*/

#ifndef	WBC_POSTURE_BEHAVIOR_HPP
#define WBC_POSTURE_BEHAVIOR_HPP

#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/motion/WholeBodyPosture.hpp>
#include <wbc/core/TaskSet.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/motion/FrictionPosture.hpp>

namespace wbc {

  /*!
    \brief This behavior describes the primitives to control the posture 
  */

  class PostureBehavior
    : public BehaviorDescription
  {
  public:
    PostureBehavior();
    
    void setFloatKey(int32_t keycode);
    void setFreezeKey(int32_t keycode);
    void addPostureKey(int32_t keycode, SAIVector const & posture);
    
    virtual TaskSet* activeTaskSet() { return activeTaskSet_; }
    virtual void onUpdate();
    
    virtual int handleKey(int keycode);
    virtual int handleSetGoal(SAIVector const & goal);
    
  protected:
    virtual void loadMovementPrimitives( RobotControlModel* ) throw(std::runtime_error);
    
    typedef std::map<int32_t, SAIVector> key_posture_t;
    key_posture_t key_posture_;
    int32_t float_key_;
    int32_t freeze_key_;
    
    TaskSet taskSetOperational_;
    TaskSet taskSetFloat_;
    TaskSet* activeTaskSet_;
    WholeBodyPosture whole_body_posture_;
    FrictionPosture friction_posture_;
  };
  
}

#endif
