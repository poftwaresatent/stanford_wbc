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

#include "tao_util.hpp"
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDNode.h>
#include <tao/dynamics/taoJoint.h>
#include <wbcnet/strutil.hpp>
#include <wbc/core/BranchingRepresentation.hpp>


namespace wbc {
  
  void mapNodesToIDs(idToNodeMap_t & idToNodeMap,
		     taoDNode * node)
    throw(std::runtime_error)
  {
    deInt id = node->getID();
    if (idToNodeMap.find( id ) != idToNodeMap.end())
      throw std::runtime_error("wbc::mapNodesToIDs(): duplicate ID " + sfl::to_string(id));
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
      link_name(_link_name),
      joint_name(_joint_name),
      limit_lower(_limit_lower),
      limit_upper(_limit_upper)
  {
  }
  
  
  tao_node_info_s::
  tao_node_info_s(tao_node_info_s const & orig)
    : id(orig.id),
      node(orig.node),
      link_name(orig.link_name),
      joint_name(orig.joint_name),
      limit_lower(orig.limit_lower),
      limit_upper(orig.limit_upper)
  {
  }
  
  
  tao_tree_info_s::
  ~tao_tree_info_s()
  {
    delete root;
  }
  
  
  tao_tree_info_s * create_tao_tree_info(BranchingRepresentation & branching)
  {
    tao_tree_info_s * tree(new tao_tree_info_s());
    tree->root = branching.rootNode();
    typedef idToNodeMap_t foo_t;
    foo_t const & foo(branching.idToNodeMap());
    int maxid(0);
    for (foo_t::const_iterator ifoo(foo.begin()); ifoo != foo.end(); ++ifoo) {
      if (ifoo->first > maxid) {
	maxid = ifoo->first;
      }
    }
    tree->info.resize(maxid+1);
    for (foo_t::const_iterator ifoo(foo.begin()); ifoo != foo.end(); ++ifoo) {
      if (ifoo->first >= 0) {
	tree->info[ifoo->first].id = ifoo->first;
	tree->info[ifoo->first].node = ifoo->second;
	tree->info[ifoo->first].link_name = "(not included in this quick hack)";
	tree->info[ifoo->first].joint_name = "(not included in this quick hack)";
	tree->info[ifoo->first].limit_lower = -1234.567;
	tree->info[ifoo->first].limit_upper = +1234.567;
      }
    }
    return tree;
  }
  
}
