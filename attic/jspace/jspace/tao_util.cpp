/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2009 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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
   \file jspace/tao_util.cpp
   \author Roland Philippsen (roland DOT philippsen AT gmx DOT net)
*/

#include "tao_util.hpp"
#include "strutil.hpp"
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDNode.h>
#include <tao/dynamics/taoJoint.h>
#include <limits>


namespace jspace {
  
  void mapNodesToIDs(idToNodeMap_t & idToNodeMap,
		     taoDNode * node)
    throw(std::runtime_error)
  {
    deInt id = node->getID();
    if (idToNodeMap.find( id ) != idToNodeMap.end())
      throw std::runtime_error("jspace::mapNodesToIDs(): duplicate ID " + sfl::to_string(id));
    idToNodeMap.insert(std::make_pair(id, node));
    
    // recurse
    for( taoDNode* p = node->getDChild(); p != NULL; p = p->getDSibling() )
      mapNodesToIDs(idToNodeMap, p);
  }
  
  
  int countNumberOfLinks(taoDNode * root)
  {
    int count(0);
    for (taoDNode * child(root->getDChild()); child != NULL; child = child->getDSibling()) {
      ++count;
      count += countNumberOfLinks(child);
    }
    return count;
  }
  
  
  int countNumberOfJoints(taoDNode * node)
  {
    int count(0);
    for (taoJoint * joint(node->getJointList()); 0 != joint; joint = joint->getNext()) {
      ++count;
    }
    for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
      count += countNumberOfJoints(child);
    }
    return count;
  }
  
  
  int countDegreesOfFreedom(taoDNode * node)
  {
    int dof(0);
    for (taoJoint * joint(node->getJointList()); 0 != joint; joint = joint->getNext()) {
      dof += joint->getDOF();
    }
    for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
      dof += countDegreesOfFreedom(child);
    }
    return dof;
  }
  
  
  double computeTotalMass(taoDNode * node)
  {
    double mass(0);
    if (node->mass()) {
      // I guess TAO nodes always have a mass, but the interface
      // returns a pointer, so maybe there are cases where there is
      // not even a zero mass? Whatever, just be paranoid and check
      // for non-NULL pointers.
      mass = *node->mass();
    }
    for (taoDNode * child(node->getDChild()); child != NULL; child = child->getDSibling()) {
      mass += computeTotalMass(child);
    }
    return mass;
  }
  
  
  tao_node_info_s::
  tao_node_info_s()
    : id(-2),
      node(0),
      joint(0),
      link_name(""),
      joint_name(""),
      limit_lower(0),
      limit_upper(0)
  {
  }
  
  
  tao_node_info_s::
  tao_node_info_s(taoDNode * _node,
		  std::string const & _link_name,
		  std::string _joint_name,
		  double _limit_lower,
		  double _limit_upper)
    : id(_node->getID()),
      node(_node),
      joint(0),
      link_name(_link_name),
      joint_name(_joint_name),
      limit_lower(_limit_lower),
      limit_upper(_limit_upper)
  {
    if (node) {
      joint = node->getJointList();
    }
  }
  
  
  tao_node_info_s::
  tao_node_info_s(tao_node_info_s const & orig)
    : id(orig.id),
      node(orig.node),
      joint(orig.joint),
      link_name(orig.link_name),
      joint_name(orig.joint_name),
      limit_lower(orig.limit_lower),
      limit_upper(orig.limit_upper)
  {
  }
  
  
  tao_tree_info_s::
  tao_tree_info_s()
    : root(0)
  {
  }
  
  
  tao_tree_info_s::
  ~tao_tree_info_s()
  {
    delete root;
  }
  
  
  bool tao_tree_info_s::
  sort()
  {
    // swap any out-of-order entries (yes yes, this is
    // suboptimal... let's just assume that in most cases they will
    // already be correctly ordered anyway)
    for (ssize_t ii(0); ii < info.size(); ++ii) {
      if (info[ii].id != ii) {
	for (ssize_t jj(ii + 1); jj < info.size(); ++jj) {
	  if (info[jj].id == ii) {
	    std::swap(info[ii], info[jj]);
	    break;
	  }
	}
      }
    }
    // check if it worked
    for (ssize_t ii(0); ii < info.size(); ++ii) {
      if (info[ii].id != ii) {
	return false;
      }
    }
    return true;
  }
  
  
  static void _recurse_create_bare_tao_tree_info(tao_tree_info_s * tree_info,
						 taoDNode * node)
  {
    tree_info->info.push_back(tao_node_info_s());
    tao_node_info_s & node_info(tree_info->info.back());
    node_info.node = node;
    node_info.joint = node->getJointList();
    node_info.id = node->getID();
    node_info.link_name = "link" + sfl::to_string(node_info.id);
    node_info.joint_name = "joint" + sfl::to_string(node_info.id);
    node_info.limit_lower = std::numeric_limits<double>::min();
    node_info.limit_upper = std::numeric_limits<double>::max();
    for (taoDNode * child(node->getDChild()); child != NULL; child = child->getDSibling()) {
      _recurse_create_bare_tao_tree_info(tree_info, child);
    }
  }
  
  
  tao_tree_info_s * create_bare_tao_tree_info(taoNodeRoot * root)
  {
    tao_tree_info_s * tree_info(new tao_tree_info_s());
    tree_info->root = root;
    for (taoDNode * child(root->getDChild()); child != NULL; child = child->getDSibling()) {
      _recurse_create_bare_tao_tree_info(tree_info, child);
    }
    return tree_info;
  }
  
  
  typedef std::map<int, int> id_counter_t;
  
  static void tao_collect_ids(taoDNode * node, id_counter_t & id_counter)
  {
    int const id(node->getID());
    id_counter_t::iterator idc(id_counter.find(id));
    if (id_counter.end() == idc) {
      id_counter.insert(std::make_pair(id, 1));
    }
    else {
      ++idc->second;
    }
    for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
      tao_collect_ids(child, id_counter);
    }
  }
  
  
  int tao_consistency_check(taoNodeRoot * root, std::ostream * msg)
  {
    if (root->getID() != -1) {
      if (msg) {
	*msg << "jspace::tao_consistency_check(): root has ID " << root->getID() << " instead of -1\n";
      }
      return 1;
    }
    id_counter_t id_counter;
    for (taoDNode * node(root->getDChild()); 0 != node; node = node->getDSibling()) {
      tao_collect_ids(node, id_counter);
    }
    int expected_id(0);
    for (id_counter_t::const_iterator idc(id_counter.begin()); idc != id_counter.end(); ++idc, ++expected_id) {
      if (idc->first != expected_id) {
	if (msg) {
	  *msg << "jspace::tao_consistency_check(): ID gap, expected "
	       << expected_id << " but encountered " << idc->first << "\n";
	}
	return 2;
      }
      if (1 != idc->second) {
	if (msg) {
	  *msg << "jspace::tao_consistency_check(): duplicate ID " << idc->first << "\n";
	}
	return 3;
      }
    }
    return 0;
  }
  
}
