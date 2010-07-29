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
       This is "the" legacy parser for SAI XML files. It is ugly old
       code that has been wrapped in the somewhat nicer interface.
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
      // std::map<std::string, SAIVector> linkToSensor_;
      // std::map<std::string, double> linkToSurfaceDepth_;
      
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
