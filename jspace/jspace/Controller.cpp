/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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
   \file jspace/Controller.cpp
   \author Roland Philippsen
*/

#include "Controller.hpp"
#include "Model.hpp"
#include <rbdl.h>

using namespace RigidBodyDynamics;


namespace jspace {
  

  void jspace_controller_info_getter_s::
  getDOFNames(Model const & model, std::vector<std::string> & names) const
  {
    int const njoints(model.getNJoints());
    names.resize(njoints);
    for (int ii(0); ii < njoints; ++ii) {
      names[ii] = model.getJointName(ii);
    }
  }
  
  
  void jspace_controller_info_getter_s::
  getDOFUnits(Model const & model, std::vector<std::string> & names) const
  {
    RigidBodyDynamics::Model const * rbdl(model.getRBDL());
    names.resize(rbdl->mJoints.size());
    for (size_t ii(0); ii < names.size(); ++ii) {
      switch (rbdl->mJoints[ii].mJointType) {
      case JointTypeRevolute:
	names[ii] = "rad";
	break;
      case JointTypePrismatic:
	names[ii] = "m";
	break;
      default:
	names[ii] = "void";
      }
    }
  }
  
  
  void jspace_controller_info_getter_s::
  getGainNames(Model const & model, std::vector<std::string> & names) const
  {
    getDOFNames(model, names);
  }
  
  
  void jspace_controller_info_getter_s::
  getLimits(Model const & model, Vector & limits_lower, Vector & limits_upper) const
  {
    model.getJointLimits(limits_lower, limits_upper);
  }
  
  
  Controller::
  Controller()
    : info_getter_(0)
  {
  }
  
  
  Controller::
  ~Controller()
  {
    delete info_getter_;
  }
  
  
  controller_info_getter_s const * Controller::
  getInfo() const
  {
    if ( ! info_getter_) {
      info_getter_ = new jspace_controller_info_getter_s();
    }
    return info_getter_;
  };
  
}
