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
   \file wbcros/Model.cpp
   \author Roland Philippsen
*/

#include "Model.hpp"
// #include "urdf_to_tao.hpp"
//#include <jspace/ros/urdf_dump.hpp>
// #include <urdf/model.h>
#include <ros/console.h>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/MobileManipulatorTaskModel.hpp>
// #include <wbc/util/dump.hpp>
#include <wbcnet/endian_mode.hpp>
// #include <saimatrix/SAIVector.h>
// #include <tao/dynamics/taoNode.h>
// #include <XmlRpcValue.h>
// #include <XmlRpcException.h>

using namespace std;


namespace wbcros {
  
  
  Model::
  Model(std::string const & param_prefix)
    : jspace::ros::Model(param_prefix),
      branching_(0),
      control_model_(0)
  {
  }
  
  
  Model::
  ~Model()
  {
    for (size_t ii(0); ii < task_model_pool_.size(); ++ii) {
      delete task_model_pool_[ii];
    }
    // Arghl how much time you can loose when people don't use boost::shared_ptr<>!
    delete control_model_;	// this dude also deletes branching_ for us...
    // The tao_trees_[ii] get deleted by our superclass
  }
  
  
  void Model::
  initFromURDF(ros::NodeHandle &nn, urdf::Model const & urdf,
	       size_t task_model_pool_size,
	       size_t n_tao_trees) throw(std::runtime_error)
  {
    // NOTE: this method makes sure that we get at least one TAO tree,
    // which we blindly use below...
    jspace::ros::Model::initFromURDF(nn, urdf, n_tao_trees);
    afterInit(task_model_pool_size);
  }


  void Model::
  afterInit(size_t task_model_pool_size) throw(std::runtime_error)
  {
    SAIVector const gravity(gravity_, 3);
    std::vector<std::string> link_name;
    std::vector<std::string> joint_name;
    std::vector<double> limit_lower;
    std::vector<double> limit_upper;
    for (size_t ii(0); ii < tao_trees_[0]->info.size(); ++ii) {
      link_name.push_back(tao_trees_[0]->info[ii].link_name);
      joint_name.push_back(tao_trees_[0]->info[ii].joint_name);
      limit_lower.push_back(tao_trees_[0]->info[ii].limit_lower);
      limit_upper.push_back(tao_trees_[0]->info[ii].limit_upper);
    }
    branching_ = wbc::BranchingRepresentation::create(tao_trees_[0]->root,
						      &gravity,
						      0, // default unactuation matrix is identity
						      tao_root_name_,
						      &link_name,
						      &joint_name,
						      &limit_lower,
						      &limit_upper);
    if ( ! branching_) {
      throw runtime_error("weird, wbc::BranchingRepresentation::create() returned NULL");
    }
    ROS_INFO ("wbcros::Model::initFromURDF(): created wbc::BranchingRepresentation:\n"
	      "  number of joints:          %d\n"
	      "  number of actuated joints: %d\n"
	      "  total mass:                %g\n",
	      branching_->numJoints(),
	      branching_->numActuatedJoints(),
	      branching_->totalMass());
    if (0 == branching_->numJoints()) {
      ostringstream msg;
      msg << "wbcros::Model::initFromURDF(): no joints in wbc::BranchingRepresentation\n"
	  << "  TAO root name: " << tao_root_name_ << "\n";
      throw runtime_error(msg.str());
    }
    if (branching_->numJoints() != branching_->numActuatedJoints()) {
      ostringstream msg;
      msg << "wbcros::Model::initFromURDF(): wbc::BranchingRepresentation has "
	  << branching_->numJoints() << " joints but "
	  << branching_->numActuatedJoints() << " ACTUATED joints (should be the same number)";
      throw runtime_error(msg.str());
    }
    
    control_model_ = new wbc::RobotControlModel(branching_);
    ndof_ = branching_->numJoints();
    ndof_actuated_ = branching_->numActuatedJoints();
    nvel_ = ndof_; // one day maybe we will actually have spherical joints... then these will differ
    control_model_->getForceDimension(contact_nrows_, contact_ncols_); // well, these are (0, 0) anyway...
    
    static wbcnet::endian_mode_t const endian_mode(wbcnet::ENDIAN_NEVER_SWAP);
    for (size_t ii(0); ii < task_model_pool_size; ++ii) {
      task_model_pool_.push_back(new wbc::MobileManipulatorTaskModel(ndof_, endian_mode));
    }
  }
  
  
  void Model::
  initFromParam(ros::NodeHandle &nn, std::string const & urdf_param_name,
		size_t task_model_pool_size,
		size_t n_tao_trees) throw(std::runtime_error)
  {
    jspace::ros::Model::initFromParam(nn, urdf_param_name, n_tao_trees);
    afterInit(task_model_pool_size);
  }
  
}
