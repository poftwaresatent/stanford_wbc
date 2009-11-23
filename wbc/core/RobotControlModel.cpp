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

//==============================================================================
/*!
  \author     Luis Sentis
  \file       RobotControlModel.cpp
*/
//==============================================================================

#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/Dynamics.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/core/Contact.hpp>

namespace wbc {

  RobotControlModel::RobotControlModel( BranchingRepresentation * branching_rep ) 
  : dynamics_( NULL )
  {
    // Construct branching representation
    branchingModel_ = branching_rep;

    // Construct dynamics module and pass reference
    dynamics_ = new Dynamics();
    dynamics_->robotControlModel( this );

    // Construct kinematics module
    kinematics_ = new Kinematics();
    kinematics_->robotControlModel( this );

    // Construct contact module
    contact_ = new Contact();
    contact_->robotControlModel( this );
  }


  RobotControlModel::~RobotControlModel() {

    delete branchingModel_;
    delete dynamics_;
    delete kinematics_;
    delete contact_;
  }

}
