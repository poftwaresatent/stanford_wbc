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
   \file BRBuilder.hpp
   \author Roland Philippsen
*/

#ifndef WBC_BR_BUILDER_HPP
#define WBC_BR_BUILDER_HPP

#include <stdexcept>
#include <string>
#include <map>

class taoDNode;

namespace wbc {

  class BranchingRepresentation;
  
  /**
     Experimental approach to building TAO trees using an interface
     that is as independent as possible from other libraries. Not
     tested a lot though.
     
     The idea is create a BRBuilder, then feed the tree description to
     its various construction methods. The idea here is to decouple
     parsing mechanics from TAO instantiation logic, so that it does
     not matter whether you get the description from XML files, YAML
     files, users clicking button, or whatever. At the end of the
     creation process, you call the create() method to actually
     instantiate a wbc::BranchingRepresentation based on the
     accumulated data.
     
     \note It would be fairly easy to add different signatures for
     different frame representations. The {trans, rot-axis, rot-angle}
     form is just the most convenient for the existing XML file
     format. That's why setRoot() and linkNode() use seven doubles to
     specify the frame.
  */
  class BRBuilder
  {
  public:
    /**
       Initializes gravity to Earth's 9.81m/s/s along Z.
     */
    BRBuilder();
    
    /**
       Specify an alternative gravity. The constructor sets this to
       Earth's values though, so you might not need this method at
       all.
    */
    void setGravity(double gx, double gy, double gz);
    
    /**
       Define where the root node is positioned.  The root note is
       always considered fixed.
     */
    void setRoot(double frame_tx, double frame_ty, double frame_tz,
		 double frame_rx, double frame_ry, double frame_rz, double frame_ra)
      throw(std::runtime_error);
    
    /**
       Add a node to the tree. There is no support for fixed non-root
       joints. Neither is there support for free-floating nodes,
       although the BranchingRepresentation can handle those.
       
       \return The ID of the newly created node. You use this ID later
       in a call to linkNode() when you specify how the nodes are
       connected.
    */
    int addNode(std::string const & name,
		double com_x, double com_y, double com_z,
		double mass,
		double inertia_x, double inertia_y, double inertia_z);
    
    /**
       Register a node with its parent. Each node has one parent, but
       it can have several children (it's a tree).
     */
    void linkNode(/** As previously returned by addNode(). It's an
		      error to say -1 here (the root node has no
		      parent). */
		  int nodeID,
		  /** Use -1 if the parent is the root node. */
		  int parentID,
		  double frame_tx, double frame_ty, double frame_tz,
		  double frame_rx, double frame_ry, double frame_rz, double frame_ra)
      throw(std::runtime_error);
    
    /**
       Add a joint to a node.
       
       \todo TAO has support for damping and inertia terms for each
       joint... but both are set to zero here.
    */
    void addJoint(int nodeID,
		  std::string const & name,
		  double default_pos, double lower_limit, double upper_limit,
		  /** 'p', 'r', or 's' (upper case also OK) */
		  char type,
		  /** 'x', 'y', or 'z' (upper case also OK, ignored of type=='s')*/
		  char axis)
      throw(std::runtime_error);
    
    /**
       After constructing the nodes and joints using setRoot(),
       addNode(), linkNode(), and addJoint(), this method creates a
       BranchingRepresentation that wraps the TAO tree and provides
       some additional sugar (like mappings from joint names to
       nodes).
       
       \return A freshly allocated and initialized instance. If an
       error occurs, tthis method throws an exception.
    */
    BranchingRepresentation * create() throw(std::runtime_error);
    
  protected:
    typedef std::map<int, taoDNode*> nodemap_t;
    typedef std::map<int, std::string> stringmap_t;
    typedef std::map<int, double> valmap_t;
    
    double gx_, gy_, gz_;
    int nextID_;
    nodemap_t node_;
    stringmap_t node_name_;
    stringmap_t joint_name_;
    valmap_t default_pos_, lower_limit_, upper_limit_;
  };
  
}

#endif // WBC_BR_BUILDER_HPP
