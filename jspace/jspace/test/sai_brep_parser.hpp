/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
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
   \file sai_brep_parser.hpp
   \author Luis Sentis (copy-paste-adapted by Roland Philippsen)
*/

#ifndef JSPACE_TESTS_SAI_BREP_PARSER_HPP
#define JSPACE_TESTS_SAI_BREP_PARSER_HPP

#include <tao/matrix/TaoDeMath.h>
#include <map>
#include <stdexcept>

namespace wbc_tinyxml {
  class TiXmlElement;
}

namespace jspace {
  namespace test {
    
    class BranchingRepresentation;
    
    /**
       This is the legacy parser for SAI XML files. It is not exactly
       flexible, nor is it great code, but we keep it in order to not
       break backwards compatibility. The resulting XML format is a
       bit weird, too, but we'll just live with it. Anyway, the
       recognized tags are:
       
       - dynworld: must be the root element.

       - baseNode: must be the first child of the root
         element. Elements that come within the baseNode before the
         first jointNode will be considered values for the robot as a
         whole:
	 - robotName (probably optional): the name of the robot
	 - gravity (probably required): the comma-delimited gravity
	   acceleration vector (e.g "0.0, 0.0, -9.81")
	 - pos (probably required): comma-delimited position of the
           base node origin in global coordinates
	 - rot (probably required): comma-delimited axis-angle representation ("ex, ey,
           ez, phi") of the base frame's orientation wrt the global
           frame
	 - ID (maybe optional): identifier of the root node, but it
	   must have the value -1 (maybe there's some historical
	   reason for specifying it nevertheless --- someone could try
	   to find out what happens when you don't specify it or maybe
	   say something else than -1 here... it would probably be
	   harmless).

       - jointNode: specifies a link and the (single) joint with which
         it is connected to its parent. A node is considered a child
         if it's opening jointNode tag is within its parent's
         jointNode element. (That's also how jointNodes become
         children of the root node, because they sit inside
         baseNode). The following tags are valid within a jointNode,
         but make sure they appear before any child jointNode because
         the parser is not re-entrant and it will probably end up
         mixing a child's value with the parent's value for some if
         not all of these... which is another reason for making sure
         that you always specify all of these values, even if they are
         optional, because you might otherwise end up with a child
         that contains some values from it parent.
	 - jointName (optional): name of the joint, e.g. "shoulder-yaw"
	 - linkName (optional): name of the link, e.g. "upper-arm"
	 - upperJointLimit (optional): a single floating point value
	 - lowerJointLimit (optional): a single floating point value
	 - defaultJointPosition (optional): a single floating point value
	 - type (required): a single character specifying the type of
           joint. 'R' or 'r' stands for "revolute", 'P' or 'p'
           stands for "prismatic", and 'S' or 's' stands for
           "spherical". HOWEVER spherical joints are known to cause
           issues in at least some portions of the WBC code, so it is
           better to avoid them altogether (or spawn a
           spherical-joint-debugging side project).
	 - axis (required): a single character specifying the axis
           around which the joint rotates (for revolute joints) or
           along which it slides (for prismatic joints)... it would
           seem that this has no influence on spherical joints, but we
           do not really support them anyway, so nobody really knows
           for certain. Anyway, possible values are 'x', 'X', 'y',
           'Y', 'z', or 'Z' (with obvious meanings).
	 - mass (required): a single floating point number
           specifying the mass of the link
	 - com (required): three comma-separated floating
           point numbers specifying the position of the center of mass
           wrt to the home frame of the node
	 - inertia (required): three comma-separated
           floating-point numbers specifying the diagonal of the
           inertia matrix (i.e. "Ixx, Iyy, Izz"). Note that (i) the
           inertia is defined with respect to the center of mass, (ii)
           the axes of the COM frame are assumed to be parallel to the
           axes of the node's home frame, and (iii) it is not possible
           to specify off-diagonal elements of the inertia matrix
           (they are all assumed to be zero). This means that you
           cannot specify all possible cases for the inertia tensor,
           tough.
	 - pos (required): three comma-separated floating
           point numbers specifying the position of the origin of the
           node's home frame wrt to the home frame of its parent.
	 - rot (required): comma-delimited axis-angle representation ("ex, ey,
           ez, phi") of this node's home frame orientation wrt to its parent's frame.
	 - ID (required): node ID, which must be an integer, each node
           must have a unique ID, there must be no gaps in the node
           IDs, and the first child of the root node must have an ID
           of zero. In other words, it would be much better to
           automatically assign the IDs, for instance to also assure
           that a node's ID is lower than any of its children's, but
           presumably historical reasons lead to this quirk.
    */
    class BRParser
    {
    public:
      BranchingRepresentation * parse(std::string const & fileName) throw(std::runtime_error);
      
    private:
      /** The thing we're creating while parsing the XML file. */
      BranchingRepresentation * robot_;
      int nodeID_;
      deFrame homeF_;
      deFloat defaultJointPos_;
      deFloat upperJointLimit_;
      deFloat lowerJointLimit_;
      std::map<int, deFloat> defaultJointPosMap_;
      std::map<int, deFloat> upperJointLimitMap_;
      std::map<int, deFloat> lowerJointLimitMap_;
      int opID_;
      
      // /** \todo Probably unused... kick out please. */
      char type_;			// 'p', 'r', or 's'
      char axis_;			// 'x', 'y', or 'z'
      deFloat mass_;
      deVector3 inertia_;
      deVector3 com_;
      std::string robotName_;
      std::string jointName_;
      bool jointIsFree_;
      std::string linkName_;
      deVector3 rotAxis_;
      deFloat rotAngle_;
      
      /** Depth First Search on joint nodes. */
      void DFS_JointNodes(wbc_tinyxml::TiXmlElement *, int) throw(std::runtime_error);
    
      /** Search for child xml joint node. */
      wbc_tinyxml::TiXmlElement * getChildJointNode(wbc_tinyxml::TiXmlElement *);
      
      /** Read data xml joint nodes. */
      void exploreJointNode(wbc_tinyxml::TiXmlElement *) throw(std::runtime_error);
    
      /** Searches for the base node and creates a branching robot using DFS algorithm. */
      void exploreRobot(wbc_tinyxml::TiXmlElement *);
      
      /** Create tao node and link it to parent node */
      void  createTreeOfNodes(int nodeID,
			      std::string const & linkName, std::string const & jointName,
			      int parentNodeID, int operationalPointID, 
			      char jointType, char jointAxis, deFrame & homeF,
			      float mass,  deVector3 & inertia, deVector3 & com);
    };
    
  }
}

#endif // JSPACE_TESTS_SAI_BREP_PARSER_HPP
