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
   \file TiXmlBRParser.hpp
   \author Luis Sentis and Roland Philippsen
*/

#ifndef WBC_TIXML_PARSER_HPP
#define WBC_TIXML_PARSER_HPP

#include <wbc/parse/BRParser.hpp>
#include <tao/matrix/TaoDeMath.h>
#include <saimatrix/SAIVector.h>
#include <map>

namespace wbc_tinyxml {
  class TiXmlElement;
}

namespace wbc {
  
  using namespace wbc_tinyxml;
  
  /**
     \todo Rename to SAIParser.
   */
  class TiXmlBRParser
    : public BRParser
  {
  public:
    virtual BranchingRepresentation * parse(const std::string& fileName) throw(std::runtime_error);

  protected:
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
    
    /** \todo Probably unused... kick out please. */
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
    std::map<std::string, SAIVector> linkToSensor_;
    std::map<std::string, double> linkToSurfaceDepth_;
    
    /** Depth First Search on joint nodes. */
    void DFS_JointNodes( TiXmlElement*, int) throw(std::runtime_error);
    
    /** Search for child xml joint node. */
    TiXmlElement* getChildJointNode( TiXmlElement*);
    
    /** Read data xml joint nodes. */
    void exploreJointNode(TiXmlElement*) throw(std::runtime_error);
    
    /** Searches for the base node and creates a branching robot using DFS algorithm. */
    void exploreRobot(TiXmlElement*);
    
    void buildUnactuationMatrix();
    
    /** Create tao node and link it to parent node */
    void  createTreeOfNodes(int nodeID,
			    std::string const & linkName, std::string const & jointName,
			    int parentNodeID, int operationalPointID, 
			    char jointType, char jointAxis, deFrame & homeF,
			    float mass,  deVector3 & inertia, deVector3 & com);

  };
  
}

#endif // WBC_TIXML_PARSER_HPP
