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
   \file TiXmlBRParser.cpp
   \author Luis Sentis and Roland Philippsen
*/

#include "TiXmlBRParser.hpp"
#include <wbc/core/BranchingRepresentation.hpp>
#include <tao/dynamics/tao.h>
#include <wbc_tinyxml/wbc_tinyxml.h>

using namespace wbc_tinyxml;


namespace wbc {
  
  
  BranchingRepresentation * TiXmlBRParser::
  parse(const string& fileName)
    throw(std::runtime_error)
  {
    TiXmlDocument doc;
    doc.LoadFile(fileName.c_str());
    if (doc.Error())
      throw runtime_error("wbc::TiXmlBRParser::parse(" + fileName
			  + "): tinyxml error: " + doc.ErrorDesc());
    
    TiXmlElement* rootElement = doc.RootElement();
    string tag = rootElement->Value();
    if ("dynworld" != tag)
      throw runtime_error("wbc::TiXmlBRParser::parse(" + fileName
			  + "): root element is <" + tag
			  + "> but should be <dynworld>");
    
    robot_ = new BranchingRepresentation();
    defaultJointPosMap_.clear();
    upperJointLimitMap_.clear();
    lowerJointLimitMap_.clear();
    linkToSensor_.clear();
    linkToSurfaceDepth_.clear();
    exploreRobot(rootElement->FirstChildElement());
    
    for (map<string, SAIVector>::iterator ii(linkToSensor_.begin());
	 ii != linkToSensor_.end(); ++ii) {
      taoDNode * node(robot_->findLink(ii->first));
      if ( ! node)
	throw runtime_error("wbc::TiXmlBRParser::parse(" + fileName
			    + "): node `" + ii->first + "' did not make it into robot (bug?)");
      robot_->linkToSensorMap_.insert(make_pair(node, ii->second));
    }
    
    for (map<string, double>::iterator ii(linkToSurfaceDepth_.begin());
	 ii != linkToSurfaceDepth_.end(); ++ii) {
      taoDNode * node(robot_->findLink(ii->first));
      if ( ! node)
	throw runtime_error("wbc::TiXmlBRParser::parse(" + fileName
			    + "): node `" + ii->first + "' did not make it into robot (bug?)");
      robot_->linkToSurfaceDepthMap_.insert(make_pair(node, ii->second));
    }
    
    return robot_;
  }
  
  
  void TiXmlBRParser::
  exploreRobot(TiXmlElement * element)
  {
    // get the value associated to the element
    string tag = element->Value();

    if( strcmp(tag.c_str(), "baseNode") == 0 ) {

      // Explores root node
      exploreJointNode(element);

      // Assign -1 to root node ID
      nodeID_ = -1;

      // Create tao root node
      robot_->rootNode_ = new taoNodeRoot(&homeF_);
      robot_->rootNode_->setIsFixed(1);
      robot_->rootNode_->setID( (deInt) -1 );
      
      // Get the first joint node
      TiXmlElement* nextJNPtr = getChildJointNode( element->FirstChildElement() );

      // Create recursively all tao nodes (Depth First Search)
      DFS_JointNodes( nextJNPtr, nodeID_ );

      // Map tao nodes to joint IDs
      mapNodesToIDs(robot_->idToNodeMap_, robot_->rootNode_);
      
      // Mirror stuff like default joint positions
      robot_->defaultJointPosVec_.setSize( robot_->numJoints_ );
      robot_->upperJointLimitVec_.setSize( robot_->numJoints_ );
      robot_->lowerJointLimitVec_.setSize( robot_->numJoints_ );
      for (int count(0); count < robot_->numJoints_; ++count) {
	robot_->defaultJointPosVec_[count] = defaultJointPosMap_[count];
	robot_->upperJointLimitVec_[count] = upperJointLimitMap_[count];
	robot_->lowerJointLimitVec_[count] = lowerJointLimitMap_[count];
      }
      
      // Initialize tao dynamics
      taoDynamics::initialize(robot_->rootNode_);
    } 
  }
  
  
  void TiXmlBRParser::
  DFS_JointNodes(TiXmlElement * jointNodePtr, int parentNodeID)
    throw(std::runtime_error)
  {
    // XXXX to do: verify that some minimum amount of values have been
    // set (e.g. ID, linkName, and jointName).
    exploreJointNode( jointNodePtr ); // here gets new ID, linkName, jointName, etc
    robot_->numJoints_++;
    buildUnactuationMatrix();
  
    // create tao node
    createTreeOfNodes(nodeID_, linkName_, jointName_, parentNodeID,
		      opID_, type_, axis_, homeF_, mass_, inertia_, com_);

    // Get joint node child
    TiXmlElement* childJNPtr = getChildJointNode( jointNodePtr->FirstChildElement() );

    // Scan children
    if ( childJNPtr ) {

      DFS_JointNodes( childJNPtr, nodeID_ );
    }

    // Get joint node sibling
    jointNodePtr = jointNodePtr->NextSiblingElement();

    // Scan siblings
    if ( jointNodePtr ) {

      DFS_JointNodes( jointNodePtr, parentNodeID );
    }
  }
  
  
  void TiXmlBRParser::
  exploreJointNode(TiXmlElement * element)
    throw(std::runtime_error)
  {
    element = element->FirstChildElement();
    string tag = element->Value();
    double x,y,z;
    double val;

    // reset values
    homeF_.translation().zero();
    homeF_.rotation().identity();
    com_.zero();

    while ( element && strcmp( tag.c_str(), "jointNode" ) != 0 ) {

      // found tag /robotName
      if( strcmp( tag.c_str(), "robotName" ) == 0 )
	robotName_ = element->FirstChild()->Value();
    
      // FOUND TAG "gravity":
      if ( strcmp( tag.c_str(), "gravity" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf,%lf,%lf",&x,&y,&z);
	robot_->grav_[0] = x;
	robot_->grav_[1] = y;
	robot_->grav_[2] = z;
      }

      // FOUND TAG "/JOINTNAME":
      if ( strcmp( tag.c_str(), "jointName") == 0 )
	{
	  jointName_ = element->FirstChild()->Value();
	  jointIsFree_ = false;
	  if( jointName_.find("free",0) != std::string::npos ) jointIsFree_ = true;
	}

      // FOUND TAG "/LINKNAME":
      else if ( strcmp( tag.c_str(), "linkName") == 0 )
	{
	  linkName_ = element->FirstChild()->Value();
	}

      // FOUND TAG "forceSensor": // relative position wrt parent node's
      // joint frame
      if ( strcmp( tag.c_str(), "forceSensor" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf,%lf,%lf",&x,&y,&z);
	double pos[] = {x,y,z};
	linkToSensor_[linkName_] = SAIVector(pos,3);
      } 

      // FOUND TAG "surfaceDepth": // relative depth with respect to joint
      // joint frame
      if ( strcmp( tag.c_str(), "surfaceDepth" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf",&z);
	linkToSurfaceDepth_[linkName_] = z;
      } 

    
      // FOUND TAG "ID":
      if ( strcmp( tag.c_str(), "ID" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%d",&nodeID_);
      }

      // FOUND TAG "pos":
      if ( strcmp( tag.c_str(), "pos" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf,%lf,%lf",&x,&y,&z);
	homeF_.translation().set((deFloat)x, (deFloat)y, (deFloat)z);
      }

      // Found tag "defaultJointPosition":
      if ( strcmp( tag.c_str(), "defaultJointPosition" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	float blah;		// more robust wrt Float being double or float
	sscanf(str, "%f", &blah);	// no one ever checks retvals, do they???
	defaultJointPos_ = blah;
      }
      // Found tag "upperJointLimit":
      if ( strcmp( tag.c_str(), "upperJointLimit" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	float blah;		// more robust wrt Float being double or float
	sscanf(str, "%f", &blah);	// no one ever checks retvals, do they???
	upperJointLimit_ = blah;
      }

      // Found tag "lowerJointLimit":
      if ( strcmp( tag.c_str(), "lowerJointLimit" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	float blah;		// more robust wrt Float being double or float
	sscanf(str, "%f", &blah);	// no one ever checks retvals, do they???
	lowerJointLimit_ = blah;
      }

      // FOUND TAG "rot":
      else if ( strcmp( tag.c_str(), "rot" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf,%lf,%lf,%lf",&x,&y,&z,&val);
	rotAxis_.set((deFloat)x,(deFloat)y,(deFloat)z);
	rotAngle_ = (deFloat)val;
	homeF_.rotation().set( rotAxis_, rotAngle_ );
      }

      // FOUND TAG "opID":
      else if ( strcmp( tag.c_str(), "opID" ) == 0 ) {
	int opID(0);
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%d",&opID);
	opID_ = opID;
      }

      // FOUND TAG "type":
      else if ( strcmp( tag.c_str(), "type" ) == 0 ) {
	string const typeJoint(element->FirstChild()->Value());
	switch (typeJoint[0]) {
	case 'p': case 'P': type_ = 'p'; break;
	case 'r': case 'R': type_ = 'r'; break;
	case 's': case 'S': type_ = 's'; break;
	default:
	  throw std::runtime_error("wbc::TiXmlBRParser::exploreJointNode(): invalid <type> `"
				   + typeJoint + "' (should be P, R, or S)");
	}
      }

      // FOUND TAG "axis":
      else if ( strcmp( tag.c_str(), "axis" ) == 0 ) {
	string const typeAxis(element->FirstChild()->Value());
	switch (typeAxis[0]) {
	case 'x': case 'X': axis_ = 'x'; break;
	case 'y': case 'Y': axis_ = 'y'; break;
	case 'z': case 'Z': axis_ = 'z'; break;
	default:
	  throw std::runtime_error("wbc::TiXmlBRParser::exploreJointNode(): invalid <axis> `"
				   + typeAxis + "' (should be X, Y, or Z)");
	}
      }
      
      // FOUND TAG "mass":
      else if ( strcmp( tag.c_str(), "mass" ) == 0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf",&val);
	mass_ = (deFloat)val;
	robot_->totalMass_ += mass_;
      }

      // FOUND TAG "inertia":
      else if ( strcmp( tag.c_str(), "inertia" ) ==  0 ) {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf,%lf,%lf",&x,&y,&z);
	inertia_.set((deFloat)x,(deFloat)y,(deFloat)z);
      }

      // FOUND TAG "com":
      else if ( strcmp( tag.c_str(), "com" ) == 0 )  {
	const char* str = element->FirstChild()->Value();
	sscanf(str, "%lf,%lf,%lf",&x,&y,&z);
	com_.set((deFloat)x,(deFloat)y,(deFloat)z);
      }

      // Move to the next sibling
      element = element->NextSiblingElement();
      if( element ) tag = element->Value();
    }

    // fill default position map 
    defaultJointPosMap_[nodeID_] = defaultJointPos_;
  
    // fill joint limit  maps 
    upperJointLimitMap_[nodeID_] = upperJointLimit_;
    lowerJointLimitMap_[nodeID_] = lowerJointLimit_;
  }


  TiXmlElement * TiXmlBRParser::
  getChildJointNode(TiXmlElement * element)
  {
    // get the value the element
    string tag = element->Value();

    // get the value associated to its child
    string str = element->FirstChild()->Value();

    while ( element && strcmp( tag.c_str(), "jointNode" ) != 0 ) {

      // Move to the next chidren
      element = element->NextSiblingElement();

      if (element) {
	tag = element->Value();		
	str = element->FirstChild()->Value();
      }
    }

    return element;
  }


  void TiXmlBRParser::
  buildUnactuationMatrix()
  {
    // append zero column to match sizes
    if( robot_->unactuationMatrix_.row() != 0 ) {

      SAIMatrix col( robot_->unactuationMatrix_.row(), 1);
      col.zero();
      robot_->unactuationMatrix_.appendHorizontally( col );
    }

    // append new selection row
    SAIMatrix row( 1, robot_->numJoints_ );
    row.zero();
    row.elementAt( 0, nodeID_ ) = 1.0;

    if( !jointIsFree_ )
      robot_->unactuationMatrix_.appendVertically( row );
  }
  
  
  /** Create tao node and link it to parent node */
  void  TiXmlBRParser::
  createTreeOfNodes(int nodeID,
		    std::string const & linkName,
		    std::string const & jointName,
		    int parentNodeID,
		    int operationalPointID, 
		    char jointType,
		    char jointAxis,
		    deFrame & homeF,
		    float mass, 
		    deVector3 & inertia,
		    deVector3 & com)
  {
    // Find parent node
    taoDNode* parentNode = robot_->findNodeID( robot_->rootNode_, parentNodeID );

    // Create new node and link it with parent node
    taoNode* new_child_node = new taoNode(parentNode, &homeF);

    // Set up link properties
    deFrame comFrameLocal; comFrameLocal.identity();
    comFrameLocal.translation() = com;

    deMassProp tmp_m;
    tmp_m.inertia(inertia[0],inertia[1],inertia[2],&comFrameLocal);
    tmp_m.mass(mass,&comFrameLocal);
    tmp_m.get(new_child_node->mass(),new_child_node->center(),new_child_node->inertia());
    new_child_node->setID((deInt) nodeID);

    // Create new joint
    taoAxis tmp_axis; 
    if(jointAxis == 'x') tmp_axis = TAO_AXIS_X; 
    else if(jointAxis == 'y') tmp_axis = TAO_AXIS_Y; 
    else tmp_axis = TAO_AXIS_Z;	// already thrown in case it's neither x, y, or z
    
    taoJoint* joint(0);
    switch(jointType) {
    case 'p':
      joint = new taoJointPrismatic(tmp_axis);
      joint->setDVar(new taoVarDOF1);
      break;
    case 'r':
      joint = new taoJointRevolute(tmp_axis);
      joint->setDVar(new taoVarDOF1);
      break;
    case 's':
      joint = new taoJointSpherical();
      joint->setDVar(new taoVarSpherical); //?
      break;
    default:
      // Should probably throw an exception or so, I do not believe we
      // actually support custom joint types... but I also think it
      // would have thrown earlier anyway.
      break;
    }

    joint->reset();
    joint->setDamping(0.0);
    joint->setInertia(0.0);

    new_child_node->addJoint(joint); 

    new_child_node->addABNode();

    string const canonLinkName(BranchingRepresentation::canonicalLinkName(linkName));
    robot_->linkNameToNodeMap_[ linkName ] = new_child_node;
    robot_->linkNameToNodeMap_[ canonLinkName ] = new_child_node;
    //     robot_->linkTagToIDMap_[ robot_->linkTag_ ] = nodeID;
    //     robot_->IDToLinkTagMap_[ nodeID ] = robot_->linkTag_;
    string const canonJointName(BranchingRepresentation::canonicalJointName(jointName));
    robot_->jointNameToNodeMap_[ jointName_ ] = new_child_node;
    robot_->jointNameToNodeMap_[ canonJointName ] = new_child_node;
    //    robot_->jointTagToIDMap_[ robot_->jointTag_ ] = nodeID;
  }
  
}
