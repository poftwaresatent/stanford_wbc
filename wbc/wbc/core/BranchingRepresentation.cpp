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

//==========================================================================================
/*!
  \author			Luis Sentis
  \file				BranchingRepresentation.cpp
*/
//==========================================================================================

#include <wbc/core/BranchingRepresentation.hpp>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoJoint.h> 
#include <tao/utility/TaoDeMassProp.h>
#include <iostream>
#include <sstream>
#include <limits>

using namespace std;


namespace wbc {

  const SAIVector BranchingRepresentation::zero3 = SAIVector(3);
  
  
  BranchingRepresentation::
  BranchingRepresentation()
    : numJoints_(0), totalMass_(0.0), grav_(3)
  {
  }


  /** \todo how about deleting rootNode_ and tons of other stuff? */
  BranchingRepresentation::
  ~BranchingRepresentation()
  {
  }
  
  
  BranchingRepresentation * BranchingRepresentation::
  create(taoNodeRoot * tao_root,
	 SAIVector const * opt_gravity,
	 SAIMatrix const * opt_unactuation_matrix,
	 std::string const & opt_root_name,
	 std::vector<std::string> const * opt_link_names,
	 std::vector<std::string> const * opt_joint_names,
	 std::vector<double> const * opt_joint_limits_lower,
	 std::vector<double> const * opt_joint_limits_upper)
    throw(std::runtime_error)
  {
    BranchingRepresentation * robot(new BranchingRepresentation());
    
    try {
      robot->rootNode_ = tao_root;
      robot->numJoints_ = jspace::countNumberOfLinks(tao_root);
      robot->totalMass_ = jspace::computeTotalMass(tao_root);
      
      if (opt_gravity) {
	robot->grav_ = *opt_gravity;
      }
      else {
	robot->grav_[0] = 0;
	robot->grav_[1] = 0;
	robot->grav_[2] = -9.81;
      }
      
      if (opt_unactuation_matrix) {
	if (opt_unactuation_matrix->row() > opt_unactuation_matrix->column()) {
	  ostringstream msg;
	  msg << "wbc::BranchingRepresentation::create(): invalid " << opt_unactuation_matrix->row()
	      << "x" << opt_unactuation_matrix->column() << " unactuation matrix (more rows than columns)";
	  throw runtime_error(msg.str());
	}
	if (opt_unactuation_matrix->column() > robot->numJoints_) {
	  ostringstream msg;
	  msg << "wbc::BranchingRepresentation::create(): invalid " << opt_unactuation_matrix->row()
	      << "x" << opt_unactuation_matrix->column() << " unactuation matrix (robot has only " << robot->numJoints_
	      << " joints)";
	  throw runtime_error(msg.str());
	}
	robot->unactuationMatrix_ = *opt_unactuation_matrix;
      }
      else {
	robot->unactuationMatrix_.identity(robot->numJoints_);
      }
      
      jspace::mapNodesToIDs(robot->idToNodeMap_, tao_root);
      
      if (opt_root_name.empty()) {
	robot->setRootName("root");
      }
      else {
	robot->setRootName(opt_root_name);
      }
      
      if (opt_link_names) {
	robot->setLinkNames(*opt_link_names);
      }
      
      if (opt_joint_names) {
	robot->setJointNames(*opt_joint_names);
      }
      
      if (opt_joint_limits_lower && (opt_joint_limits_lower->size() != static_cast<size_t>(robot->numJoints_))) {
	ostringstream msg;
	msg << "wbc::BranchingRepresentation::create(): opt_joint_limits_lower has " << opt_joint_limits_lower->size()
	    << " entries but the robot has "  << robot->numJoints_ << " joints";
	throw runtime_error(msg.str());
      }
      if (opt_joint_limits_upper && (opt_joint_limits_upper->size() != static_cast<size_t>(robot->numJoints_))) {
	ostringstream msg;
	msg << "wbc::BranchingRepresentation::create(): opt_joint_limits_upper has " << opt_joint_limits_upper->size()
	    << " entries but the robot has "  << robot->numJoints_ << " joints";
	throw runtime_error(msg.str());
      }
      
      robot->defaultJointPosVec_.setSize(robot->numJoints_);
      robot->upperJointLimitVec_.setSize(robot->numJoints_);
      robot->lowerJointLimitVec_.setSize(robot->numJoints_);
      for (int count(0); count < robot->numJoints_; ++count) {
	robot->defaultJointPosVec_[count] = 0;
	if (opt_joint_limits_lower) {
	  robot->lowerJointLimitVec_[count] = (*opt_joint_limits_lower)[count];
	}
	else {
	  robot->lowerJointLimitVec_[count] = std::numeric_limits<double>::min();
	}
	if (opt_joint_limits_upper) {
	  robot->upperJointLimitVec_[count] = (*opt_joint_limits_upper)[count];
	}
	else {
	  robot->upperJointLimitVec_[count] = std::numeric_limits<double>::max();
	}
      }
      
      taoDynamics::initialize(tao_root); // Does this break in case someone already called this earlier?
    }
    
    catch (std::runtime_error const & ee) {
      delete robot;
      throw ee;
    }
    
    return robot;
  }
  
  
  const SAIVector& 
  BranchingRepresentation::forceSensorWRTJointFrame( taoDNode const * node )
    const
  {
    std::map<taoDNode const *, SAIVector>::const_iterator
      ii(linkToSensorMap_.find(node));
    if (ii == linkToSensorMap_.end())
      return zero3;
    return ii->second;
  }
  
  
  double 
  BranchingRepresentation::surfaceDepth( taoDNode const * node )
    const
  {
    std::map<taoDNode const *, double>::const_iterator
      ii(linkToSurfaceDepthMap_.find(node));
    if (ii == linkToSurfaceDepthMap_.end())
      return 0;
    return ii->second;
  }


  taoDNode* 
  BranchingRepresentation::findNodeID( taoDNode* node, int id ) {

    deInt tmpID = node->getID();
    if( tmpID == id ) return node;
    taoDNode* foundNode = NULL;	
    for( taoDNode* n = node->getDChild(); n!= NULL && foundNode == NULL; n = node->getDSibling() )
      foundNode = findNodeID( n, id );
    return foundNode;
  }
  
  
  /**
     \todo Reimplement with a dictionary.
  */
  string
  BranchingRepresentation::__canonicalJointName( const string &jointName ) {

    string tag("");

    if( jointName == "shoulder-yaw" )
      tag = "Shoulder_Yaw";
    else if( jointName == "shoulder-pitch" )
      tag = "Shoulder_Pitch";
    else if( jointName == "shoulder-roll" )
      tag = "Shoulder_Roll";
    else if( jointName == "wrist_pitch" )
      tag = "Wrist_Pitch";
    
    return tag;
  }


  /**
     \todo Reimplement with a dictionary.
  */
  string
  BranchingRepresentation::__canonicalLinkName( const string &linkName ) {

    string tag("");

    if( linkName == "upper-arm" )
      tag = "Upper_Arm";
    else if( linkName == "lower-arm" )
      tag = "Lower_Arm";
    else if( linkName == "end-effector" ) 
      tag = "End_Effector";
    else if( linkName == "right-hand" ) 
      tag = "Right_Hand";
    else if( linkName == "left-hand" ) 
      tag = "Left_Hand";
    else if( linkName == "hip" ) 
      tag = "Hip";
    else if( linkName == "right-foot" ) 
      tag = "Right_Foot";
    else if( linkName == "left-foot" ) 
      tag = "Left_Foot";
    else if( linkName == "right-lower-leg" ) 
      tag = "Right_Lower_Leg";
    else if( linkName == "left-lower-leg" ) 
      tag = "Left_Lower_Leg";
    
    return tag;
  }
  
  
  taoDNode * BranchingRepresentation::
  findLink( std::string const & name )
  {
    std::map<std::string, taoDNode*>::iterator ii(linkNameToNodeMapWithAliases_.find(name));
    if (ii == linkNameToNodeMapWithAliases_.end())
      return 0;
    return ii->second;
  }
  
  
  taoDNode * BranchingRepresentation::
  findJoint( std::string const & name )
  {
    std::map<std::string, taoDNode*>::iterator ii(jointNameToNodeMapWithAliases_.find(name));
    if (ii == jointNameToNodeMapWithAliases_.end())
      return 0;
    return ii->second;
  }
  
  
  void BranchingRepresentation::
  setRootName(std::string const & root_name)
    throw(std::runtime_error)
  {
    idToNodeMap_t::const_iterator node(idToNodeMap_.find(-1));
    if (idToNodeMap_.end() == node) {
      throw runtime_error("wbc::BranchingRepresentation::setRootName(" + root_name + "): no root node");
    }
    linkNameToNodeMap_[root_name] = node->second;
    linkNameToNodeMapWithAliases_[root_name] = node->second;
    string const canonical_name(__canonicalLinkName(root_name));
    if ( ! canonical_name.empty()) {
      linkNameToNodeMapWithAliases_[canonical_name] = node->second;
    }
  }
  
  
  void BranchingRepresentation::
  setLinkNames(std::vector<std::string> const & link_names)
    throw(std::runtime_error)
  {
    linkNameToNodeMap_.clear();
    linkNameToNodeMapWithAliases_.clear();
    for (size_t ii(0); ii < link_names.size(); ++ii) {
      idToNodeMap_t::const_iterator node(idToNodeMap_.find(ii));
      if (idToNodeMap_.end() == node) {
	ostringstream msg;
	msg << "wbc::BranchingRepresentation::setLinkNames(): no node with ID " << ii;
	throw runtime_error(msg.str());
      }
      linkNameToNodeMap_[link_names[ii]] = node->second;
    }
  }
  
  
  void BranchingRepresentation::
  setJointNames(std::vector<std::string> const & joint_names)
    throw(std::runtime_error)
  {
    jointNameToNodeMap_.clear();
    jointNameToNodeMapWithAliases_.clear();
    for (size_t ii(0); ii < joint_names.size(); ++ii) {
      idToNodeMap_t::const_iterator node(idToNodeMap_.find(ii));
      if (idToNodeMap_.end() == node) {
	ostringstream msg;
	msg << "wbc::BranchingRepresentation::setJointNames(): no node with ID " << ii;
	throw runtime_error(msg.str());
      }
      jointNameToNodeMap_[joint_names[ii]] = node->second;
    }
  }
  
  
  void BranchingRepresentation::
  createCanonicalAliases() const
  {
    typedef std::map<std::string, taoDNode*> foot_t;
    for (foot_t::const_iterator ifoo(linkNameToNodeMap_.begin()); ifoo != linkNameToNodeMap_.end(); ++ifoo) {
      linkNameToNodeMapWithAliases_[ifoo->first] = ifoo->second;
      string const canonical_name(__canonicalLinkName(ifoo->first));
      if ( ! canonical_name.empty()) {
	linkNameToNodeMapWithAliases_[canonical_name] = ifoo->second;
      }
    }
    for (foot_t::const_iterator ifoo(jointNameToNodeMap_.begin()); ifoo != jointNameToNodeMap_.end(); ++ifoo) {
      jointNameToNodeMapWithAliases_[ifoo->first] = ifoo->second;
      string const canonical_name(__canonicalJointName(ifoo->first));
      if ( ! canonical_name.empty()) {
	jointNameToNodeMapWithAliases_[canonical_name] = ifoo->second;
      }
    }
  }
  
  
  std::map<std::string, taoDNode*> const & BranchingRepresentation::
  linkNameToNodeMap(bool withAliases) const
  {
    if (withAliases) {
      if (linkNameToNodeMapWithAliases_.empty() && ( ! linkNameToNodeMap_.empty())) {
	createCanonicalAliases();
      }
      return linkNameToNodeMapWithAliases_;
    }
    return linkNameToNodeMap_;
  }
  
  
  std::map<std::string, taoDNode*> const & BranchingRepresentation::
  jointNameToNodeMap(bool withAliases) const
  {
    if (withAliases) {
      if (jointNameToNodeMapWithAliases_.empty() && ( ! jointNameToNodeMap_.empty())) {
	createCanonicalAliases();
      }
      return jointNameToNodeMapWithAliases_;
    }
    return jointNameToNodeMap_;
  }
  
  
  jspace::tao_tree_info_s * BranchingRepresentation::
  createTreeInfo()
  {
    jspace::tao_tree_info_s * tree(new jspace::tao_tree_info_s());
    tree->root = rootNode();
    
    typedef idToNodeMap_t foo_t;
    foo_t const & foo(idToNodeMap());
    int maxid(0);
    for (foo_t::const_iterator ifoo(foo.begin()); ifoo != foo.end(); ++ifoo) {
      if (ifoo->first > maxid) {
	maxid = ifoo->first;
      }
    }
    tree->info.resize(maxid+1);
    
    typedef std::map<std::string, taoDNode*> bar_t;
    bar_t const & link_bar(linkNameToNodeMap(false));
    bar_t const & joint_bar(jointNameToNodeMap(false));
    SAIVector const & upper(upperJointLimits());
    SAIVector const & lower(lowerJointLimits());
    
    for (foo_t::const_iterator ifoo(foo.begin()); ifoo != foo.end(); ++ifoo) {
      if (ifoo->first >= 0) {
	
	tree->info[ifoo->first].id = ifoo->first;
	tree->info[ifoo->first].node = ifoo->second;
	tree->info[ifoo->first].joint = ifoo->second->getJointList();
	
	tree->info[ifoo->first].link_name = "(not found)";
	for (bar_t::const_iterator ibar(link_bar.begin()); ibar != link_bar.end(); ++ibar) {
	  if (ifoo->second == ibar->second) {
	    tree->info[ifoo->first].link_name = ibar->first;
	    break;
	  }
	}
	
	tree->info[ifoo->first].joint_name = "(not found)";
	for (bar_t::const_iterator ibar(joint_bar.begin()); ibar != joint_bar.end(); ++ibar) {
	  if (ifoo->second == ibar->second) {
	    tree->info[ifoo->first].joint_name = ibar->first;
	    break;
	  }
	}
	
	tree->info[ifoo->first].limit_lower = lower[ifoo->first];
	tree->info[ifoo->first].limit_upper = upper[ifoo->first];
      }
    }
    return tree;




  }

}

