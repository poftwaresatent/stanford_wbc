/*
 * ROS support for Stanford-WBC http://stanford-wbc.sourceforge.net/
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
   \file wbc/ros/Model.hpp
   \author Roland Philippsen
*/

#ifndef WBCROS_MODEL_HPP
#define WBCROS_MODEL_HPP

#include <string>
#include <vector>
#include <stdexcept>

namespace ros {
  class NodeHandle;
}

namespace urdf {
  class Model;
}

namespace wbc {
  // declared in <wbc/util/tao_util.hpp>
  struct tao_tree_info_s;
  
  class BranchingRepresentation;
  class RobotControlModel;
  class TaskModelBase;
}


namespace wbcros {
    
  
  class Model
  {
  public:
    std::string tao_root_param_name_;	  // default: "tao_root_name"
    std::string tao_root_name_;
    std::string active_links_param_name_; // default: "active_links"
    std::string gravity_compensated_links_param_name_; // default: "gravity_compensated_links"
    double gravity_[3];	// default: { 0, 0, -9.81 }
    std::vector<wbc::tao_tree_info_s*> tao_trees_;
    wbc::BranchingRepresentation * branching_; // filled in by conversion function, using tao_trees_[0]->root
    std::vector<std::string> gravity_compensated_links_; // from ROS parameters, checked against active links
    wbc::RobotControlModel * control_model_; // NOTE: gets deleted by ~Model()
    std::vector<wbc::TaskModelBase*> task_model_pool_;
    
    size_t ndof_, ndof_actuated_, nvel_, contact_nrows_, contact_ncols_;
    
    explicit Model(std::string const & param_prefix);
    ~Model();
    
    void initFromURDF(ros::NodeHandle &nn, urdf::Model const & urdf,
		      size_t task_model_pool_size,
		      size_t n_tao_trees) throw(std::runtime_error);

    void initFromParam(ros::NodeHandle &nn, std::string const & urdf_param_name,
		       size_t task_model_pool_size,
		       size_t n_tao_trees) throw(std::runtime_error);
  };
  
}

#endif // WBCROS_MODEL_HPP
