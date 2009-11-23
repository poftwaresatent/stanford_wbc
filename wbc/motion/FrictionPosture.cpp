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

//===========================================================================
/*!
  \file       FrictionPosture.cpp
  \author     Luis Sentis
*/
////===========================================================================

#include <wbc/motion/FrictionPosture.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/Kinematics.hpp>
#include <iostream>

namespace wbc {

  FrictionPosture::FrictionPosture(std::string const & name)
    : TaskDescription(name, 5, -1, -1, -1)
  {}


  void
  FrictionPosture::robotControlModel( RobotControlModel* robModel ) throw(std::runtime_error) {

    robModel_ = robModel;
    std::size_t n = robModel_->branching()->numJoints();
    globalJacobian_.setSize( n, n );
    globalJacobian_.identity();
  }

  void
  FrictionPosture::onUpdate() {

    velocity_ = globalJacobian_ * robModel_->kinematics()->jointVelocities();
    servoUpdate();
  } 

  void 
  FrictionPosture::servoUpdate() {

    commandAccel_ = -velocity_ * m_diffGain;

  }

}
