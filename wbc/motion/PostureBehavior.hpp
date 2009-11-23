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
  \author     Luis Sentis
  \file       PostureBehavior.hpp
*/
//
//=========================================================================
 
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

    virtual TaskSet* activeTaskSet() { return activeTaskSet_; }
    virtual void onUpdate();

    virtual int32_t handleCommand(int32_t const * codeVector,
				  size_t nCodes,
				  SAIMatrix const & matrix);

  protected:
    virtual void loadMovementPrimitives( RobotControlModel* ) throw(std::runtime_error);

  private:

    // General attributes
    TaskSet taskSetOperational_;
    TaskSet taskSetFloat_;
    TaskSet* activeTaskSet_;
    WholeBodyPosture whole_body_posture_;
    FrictionPosture friction_posture_;
    unsigned int currentState_;
    SAIVector goalPosture_;
    //SAIVector recType_;
    SAIMatrix recType_;
  };

}

#endif
