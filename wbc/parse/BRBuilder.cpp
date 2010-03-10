/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file BRBuilder.cpp
   \author Roland Philippsen
*/

#include "BRBuilder.hpp"
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbcnet/strutil.hpp>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/utility/TaoDeMassProp.h>

using namespace std;

namespace wbc {
  
  
  BRBuilder::
  BRBuilder()
    : gx_(0),
      gy_(0),
      gz_(9.81), // assuming we're on Earth seems a sound bet for the moment
      nextID_(0)
  {
  }
  
  
  void BRBuilder::
  setGravity(double gx, double gy, double gz)
  {
    gx_ = gx;
    gy_ = gy;
    gz_ = gz;
  }
  
  
  void BRBuilder::
  setRoot(double frame_tx, double frame_ty, double frame_tz,
	  double frame_rx, double frame_ry, double frame_rz, double frame_ra)
    throw(std::runtime_error)
  {
    if (node_[-1] != 0)
      throw runtime_error("wbc::BRBuilder::setRoot(): no support for multiple roots");
    deFrame frame;
    frame.translation().set(frame_tx, frame_ty, frame_tz);
    frame.rotation().set(frame_rx, frame_ry, frame_rz, frame_ra);
    taoNodeRoot * root(new taoNodeRoot(frame));
    root->setIsFixed(1);
    root->setID(-1);
    node_[-1] = root;
  }
  
  
  int BRBuilder::
  addNode(std::string const & name,
	  double com_x, double com_y, double com_z,
	  double mass,
	  double inertia_x, double inertia_y, double inertia_z)
  {
    deFrame comframe;
    comframe.identity();
    comframe.translation().set(com_x, com_y, com_z);
    deMassProp mprop;
    mprop.inertia(inertia_x, inertia_y, inertia_z, &comframe);
    mprop.mass(mass, &comframe);
    
    taoNode * node(new taoNode());
    node->setIsFixed(0);	// probably redundant, but makes it nicely explicit
    mprop.get(node->mass(), node->center(), node->inertia());
    node->addABNode();
    
    int const id(nextID_);
    ++nextID_;
    node->setID(id);
    node_name_.insert(make_pair(id, name));
    node_.insert(make_pair(id, node));
    
    return id;
  }
  
  
  void BRBuilder::
  linkNode(int nodeID,
	   int parentID,
	   double frame_tx, double frame_ty, double frame_tz,
	   double frame_rx, double frame_ry, double frame_rz, double frame_ra)
    throw(std::runtime_error)
  {
    taoDNode * parent(node_[parentID]);
    if (0 == parent)
      throw runtime_error("wbc::BRBuilder::linkNode(): invalid parentID " + sfl::to_string(parentID));
    taoNode * node(dynamic_cast<taoNode*>(node_[nodeID]));
    if (0 == node)
      throw runtime_error("wbc::BRBuilder::linkNode(): invalid nodeID " + sfl::to_string(nodeID));
    deFrame frame;
    frame.translation().set(frame_tx, frame_ty, frame_tz);
    frame.rotation().set(frame_rx, frame_ry, frame_rz, frame_ra);
    node->link(parent, &frame);
  }
  
    
  void BRBuilder::
  addJoint(int nodeID,
	   std::string const & name,
	   double default_pos, double lower_limit, double upper_limit,
	   /** 'p', 'r', or 's' (upper case also OK) */
	   char type,
	   /** 'x', 'y', or 'z' (upper case also OK, ignored of type=='s')*/
	   char axis)
    throw(std::runtime_error)
  {
    taoNode * node(dynamic_cast<taoNode*>(node_[nodeID]));
    if (0 == node)
      throw runtime_error("wbc::BRBuilder::addJoint(): invalid nodeID " + sfl::to_string(nodeID));
    
    taoAxis tao_axis;
    switch (axis) {
    case 'x': case 'X': tao_axis = TAO_AXIS_X; break;
    case 'y': case 'Y': tao_axis = TAO_AXIS_Y; break;
    case 'z': case 'Z': tao_axis = TAO_AXIS_Z; break;
    default:
      if ((type != 's') && (type != 'S'))
	throw runtime_error("wbc::BRBuilder::addJoint(): invalid axis " + sfl::to_string(axis)
			    + " (should be x, y, or z for non-spherical joints)");
    }
    
    taoJoint * joint;
    switch (type) {
    case 'p':
    case 'P':
      joint = new taoJointPrismatic(tao_axis);
      joint->setDVar(new taoVarDOF1());
      break;
    case 'r':
    case 'R':
      joint = new taoJointRevolute(tao_axis);
      joint->setDVar(new taoVarDOF1());
      break;
    case 's':
    case 'S':
      joint = new taoJointSpherical();
      joint->setDVar(new taoVarSpherical());
      break;
    default:
      throw runtime_error("wbc::BRBuilder::addJoint(): invalid type " + sfl::to_string(type)
			  + " (should be p, r, or s)");
    }
    
    joint->reset();
    joint->setDamping(0.0);
    joint->setInertia(0.0);
    
    node->addJoint(joint); 
    
    joint_name_.insert(make_pair(nodeID, name));
    default_pos_.insert(make_pair(nodeID, default_pos));
    lower_limit_.insert(make_pair(nodeID, lower_limit));
    upper_limit_.insert(make_pair(nodeID, upper_limit));
  }
  
  
  BranchingRepresentation * BRBuilder::
  create()
    throw(std::runtime_error)
  {
    if (0 == node_[-1])
      throw runtime_error("wbc::BRBuilder::create(): no root node");

    BranchingRepresentation * br(new BranchingRepresentation());

    br->rootNode_ = dynamic_cast<taoNodeRoot*>(node_[-1]);
    if (0 == br->rootNode_) {
      delete br;
      throw runtime_error("wbc::BRBuilder::create(): ouch! BUG! node_[-1] is not a taoNodeRoot");
    }
    br->numJoints_ = node_.size() - 1; // the root node does not count
    
    br->grav_.setSize(3);
    br->grav_[0] = gx_;
    br->grav_[1] = gy_;
    br->grav_[2] = gz_;
    
    br->totalMass_ = 0;
    br->defaultJointPosVec_.setSize(br->numJoints_);
    br->lowerJointLimitVec_.setSize(br->numJoints_);
    br->upperJointLimitVec_.setSize(br->numJoints_);
    
    for (nodemap_t::const_iterator in(node_.begin()); in != node_.end(); ++in) {
      int const id(in->first);
      taoDNode * node(in->second);
      br->idToNodeMap_[id] = node;
      
      if (-1 == id)
	continue;	     // skip root node for the remaining suger
      
      br->defaultJointPosVec_[id] = default_pos_[id];
      br->lowerJointLimitVec_[id] = lower_limit_[id];
      br->upperJointLimitVec_[id] = upper_limit_[id];
      
      string const node_name(node_name_[id]);
      string const node_tag(br->canonicalLinkName(node_name));
      string const joint_name(joint_name_[id]);
      string const joint_tag(br->canonicalJointName(joint_name));
      
      br->linkNameToNodeMap_[node_name] = node;
      br->linkNameToNodeMap_[node_tag] = node;
      br->jointNameToNodeMap_[joint_name] = node;
      br->jointNameToNodeMap_[joint_tag] = node;
      
      br->totalMass_ += *node->mass();
    }
    
    // all joints are actuated in this implementation
    br->unactuationMatrix_.setSize(br->numJoints_, br->numJoints_, true);
    br->unactuationMatrix_.identity();
    
    // I wonder where this should best be done...
    taoDynamics::initialize(br->rootNode_);
    
    return br;
  }
  
}
