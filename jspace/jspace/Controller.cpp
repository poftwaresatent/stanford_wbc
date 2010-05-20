/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
  getGainNames(Model const & model, std::vector<std::string> & names) const
  {
    getDOFNames(model, names);
  }
  
  
  void jspace_controller_info_getter_s::
  getLimits(Model const & model, std::vector<double> & limits_lower, std::vector<double> & limits_upper) const
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
