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
#include <urdf/model.h>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/MobileManipulatorTaskModel.hpp>
#include <wbc/ros/urdf_to_tao.hpp>
#include <wbc/util/dump.hpp>
#include <wbcnet/endian_mode.hpp>
#include <saimatrix/SAIVector.h>
#include <tao/dynamics/taoNode.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

using namespace std;


namespace wbcros {
  
  
  Model::
  Model(std::string const & param_prefix)
    : tao_root_param_name_(param_prefix + "tao_root_name"),
      active_links_param_name_(param_prefix + "active_links"),
      tao_root_node_(0),
      branching_(0),
      control_model_(0)
  {
    gravity_[0] = 0;
    gravity_[1] = 0;
    gravity_[2] = 0;
  }
  
  
  Model::
  ~Model()
  {
    for (size_t ii(0); ii < task_model_pool_.size(); ++ii) {
      delete task_model_pool_[ii];
    }
    // Arghl how much time you can loose when people don't use boost::shared_ptr<>!
    delete control_model_;	// this dude also deletes branching_ for us...
    // delete branching_;
    delete tao_root_node_; // wbc::BranchingRepresentation dtor does not do this for us...
  }
  
  
  void Model::
  initFromURDF(ros::NodeHandle &nn, urdf::Model const & urdf,
	       size_t task_model_pool_size) throw(std::runtime_error)
  {
    if (tao_root_node_ || branching_ || control_model_ || ( ! task_model_pool_.empty())) {
      throw std::runtime_error("wbcros::Model::initFromURDF(): already (partially?) initialized");
    }
    
    if ( ! nn.getParam(tao_root_param_name_, tao_root_name_)) {
      throw std::runtime_error("wbcros::Model::initFromURDF(): invalid tao_root_param_name_ \""
			       + tao_root_param_name_ + "\"");
    }
    ROS_INFO ("tao_root_name_ is `%s'", tao_root_name_.c_str());
    
    XmlRpc::XmlRpcValue active_links_value;
    if ( ! nn.getParam(active_links_param_name_, active_links_value)) {
      throw std::runtime_error("wbcros::Model::initFromURDF(): invalid active_links_param_name_ \""
			       + active_links_param_name_ + "\"");
    }
    
    urdf_to_tao::ActiveLinkFilter link_filter;
    link_filter.AddLink(tao_root_name_);
    try {
      std::string foo;
      for (int ii(0); ii < active_links_value.size(); ++ii) {
	foo = static_cast<std::string const &>(active_links_value[ii]);
	link_filter.AddLink(foo);
	ROS_INFO ("added active link `%s'", foo.c_str());
      }
    }
    catch (XmlRpc::XmlRpcException const & ee) {
      std::ostringstream msg;
      msg << "wbcros::Model::initFromURDF():"
	  << " XmlRpcException while reading active links: "
	  << ee.getMessage();
      throw std::runtime_error(msg.str());
    }
    
    link_name_.clear();
    joint_name_.clear();
    tao_root_node_ = urdf_to_tao::convert(urdf,
					  tao_root_name_,
					  link_filter,
					  &link_name_,
					  &joint_name_);
    /* XXXX to do: if info is enabled... */ {
      std::ostringstream msg;
      msg << "converted URDF to TAO\n";
      wbc::dump_tao_tree(msg, tao_root_node_, "  ", false, &link_name_, &joint_name_);
      ROS_INFO (msg.str().c_str());
    }
    
    SAIVector const gravity(gravity_, 3);
    branching_ = wbc::BranchingRepresentation::create(tao_root_node_,
						      &gravity,
						      0, // default unactuation matrix is identity
						      tao_root_name_,
						      &link_name_,
						      &joint_name_);
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
      throw runtime_error("wbcros::Model::initFromURDF(): no joints in wbc::BranchingRepresentation");
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
		size_t task_model_pool_size) throw(std::runtime_error)
  {
    std::string urdf_string;
    if ( ! nn.getParam(urdf_param_name, urdf_string)) {
      throw runtime_error("wbcros::Model::initFromParam(): invalid urdf_param_name \""
			  + urdf_param_name + "\"");
    }
    TiXmlDocument urdf_xml;
    urdf_xml.Parse(urdf_string.c_str());
    TiXmlElement * urdf_root(urdf_xml.FirstChildElement("robot"));
    if ( ! urdf_root) {
      throw runtime_error("wbcros::Model::initFromParam(): no <robot> element in urdf_param_name \""
			  + urdf_param_name + "\"");
    }
    urdf::Model urdf_model;
    if ( ! urdf_model.initXml(urdf_root)) {
      throw runtime_error("wbcros::Model::initFromParam(): initXml() failed on urdf_param_name \""
			  + urdf_param_name + "\"");
    }
    initFromURDF(nn, urdf_model, task_model_pool_size);
  }
  
}
