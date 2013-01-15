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
   \file sai_brep.cpp
   \author Luis Sentis (copy-paste-adapted by Roland Philippsen)
*/

#include "sai_brep.hpp"
#include <tao/dynamics/taoNode.h>

using namespace std;

namespace jspace {
  namespace test {
    
    
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
  
    
    taoDNode* 
    BranchingRepresentation::findNodeID( taoDNode* node, int id ) {

      deInt tmpID = node->getID();
      if( tmpID == id ) return node;
      taoDNode* foundNode = NULL;	
      for( taoDNode* n = node->getDChild(); n!= NULL && foundNode == NULL; n = node->getDSibling() )
	foundNode = findNodeID( n, id );
      return foundNode;
    }
  
  
    jspace::tao_tree_info_s * BranchingRepresentation::
    createTreeInfo()
    {
      jspace::tao_tree_info_s * tree(new jspace::tao_tree_info_s());
      tree->root = rootNode_;
    
      typedef idToNodeMap_t foo_t;
      foo_t const & foo(idToNodeMap_);
      int maxid(0);
      for (foo_t::const_iterator ifoo(foo.begin()); ifoo != foo.end(); ++ifoo) {
	if (ifoo->first > maxid) {
	  maxid = ifoo->first;
	}
      }
      tree->info.resize(maxid+1);
    
      typedef std::map<std::string, taoDNode*> bar_t;
      bar_t const & link_bar(linkNameToNodeMap_);
      bar_t const & joint_bar(jointNameToNodeMap_);
      jspace::Vector const & upper(upperJointLimitVec_);
      jspace::Vector const & lower(lowerJointLimitVec_);
    
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
}
