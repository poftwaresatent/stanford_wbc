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
   \file jspace/tao_util.hpp
   \author Roland Philippsen (roland DOT philippsen AT gmx DOT net)
*/

#ifndef JSPACE_TAO_UTIL_H
#define JSPACE_TAO_UTIL_H

#include <stdexcept>
#include <string>
#include <vector>
#include <map>


class taoNodeRoot;
class taoDNode;
class taoJoint;


namespace jspace {

  /**
     \note TAO supports multiple joints per link, but all use cases so
     far seem to require that exactly one joint sits between two
     links, so we treat joint names just as link names until further
     notice.
  */
  struct tao_node_info_s {
    tao_node_info_s();
    tao_node_info_s(taoDNode * node, std::string const & link_name,
		    std::string joint_name, double limit_lower, double limit_upper);
    tao_node_info_s(tao_node_info_s const & orig);
    
    int id;
    taoDNode * node;
    taoJoint * joint;
    std::string link_name;
    std::string joint_name;
    double limit_lower;
    double limit_upper;
  };
  
  
  struct tao_tree_info_s {
    tao_tree_info_s();
    
    /** deletes the taoNodeRoot. */
    virtual ~tao_tree_info_s();
    
    /** Iterates over the info vector, making sure that each node sits
	at the position specified by its ID. Also does a sanity check
	afterwards, so don't ignore the return value. See also
	tao_consistency_check().
	
	\return true if the sort succeeded (info[ii].id == id for all
	nodes), false if something went wrong (this happens e.g. when
	you have duplicate IDs or gaps in the sequence from 0..N-1).
    */
    bool sort();
    
    taoNodeRoot * root;
    typedef std::vector<tao_node_info_s> node_info_t;
    node_info_t info;
  };
  
  
  /**
     Create a minimal tao_tree_info_s from a TAO tree. The created
     structure will use dummy names like "joint12" and humongous
     limits.
  */
  tao_tree_info_s * create_bare_tao_tree_info(taoNodeRoot * root);
  
  
  /**
     Run a consistency check on a TAO tree. For the time being, this
     simply checks that each node has a unique ID, that the IDs range
     from 0 to N-1, and that the root has an ID of -1.
     
     \return 0 on success, 1 if the root node's ID is wrong, 2 if
     there's a gap in the ID sequence, and 3 if there's a duplicate
     ID.
   */
  int tao_consistency_check(/** The TAO tree which should be checked. */
			    taoNodeRoot * root,
			    /** An optional pointer to a stream, where
				error messages get printed in case you
				specify a non-NULL pointer here. */
			    std::ostream * msg);
  
  
  typedef std::map<int, taoDNode *> idToNodeMap_t;
  
  /**
     Create a map between tao nodes and IDs. The \c idToNodeMap is not
     cleared for you: use this function to append to an existing map,
     or clear the map yourself prior to use.
     
     \note Throws a \c runtime_error in case there is a duplicate ID
  */
  void mapNodesToIDs(idToNodeMap_t & idToNodeMap,
		     taoDNode * node)
    throw(std::runtime_error);
  
  
  /**
     Count the total number of links connected to the given node,
     following all children in to the leaf nodes. This number does NOT
     include the given link (because usually you will call this on the
     TAO root node in order to figure out how many degrees of freedom
     the robot has, in which case you do not count the root itself).
  */
  int countNumberOfLinks(taoDNode * root);
  
  
  /**
     Count the total number of joints attached to the given node and
     all its descendants.
  */
  int countNumberOfJoints(taoDNode * node);
  
  
  /**
     Count the total number of degrees of freedom of all the joints
     attached to the given node and all its descendants.
  */
  int countDegreesOfFreedom(taoDNode * node);
  
  
  /**
     Sum up the mass of the given node plus all its descendants.
  */
  double computeTotalMass(taoDNode * node);
  
}

#endif // JSPACE_TAO_UTIL_H
