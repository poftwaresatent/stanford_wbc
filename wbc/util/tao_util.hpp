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

#ifndef WBC_TAO_UTIL_H
#define WBC_TAO_UTIL_H

#include <stdexcept>
#include <map>

class taoDNode;

namespace wbc {
  
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

#endif // WBC_TAO_UTIL_H
