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

#ifndef SROBOTLINKSTRUCTURES_HPP_
#define SROBOTLINKSTRUCTURES_HPP_

//NOTE TODO Substitute these with the appropriate headers after this is added to the build system
#include <map>
#include <list>
#include <string>
#include <vector>

#include "SGlobalRobotDS.hpp"

#ifdef ROBARCH_IS_WINDOWS_
  #include "PrVector3.h"
  #include "PrMatrix3.h"

  //Arachi library for windows instead of the tao library.
  #include "DeDNode.h"
  //(Quite conveniently) DeNode resides in DeBaseNode! :)
  #include "DeBaseNode.h"
  #include "DeDynamics.h"
  #include "DeJoint.h"
  #include "DeFrame.h"

  class DeDNode;
  class DeNode;
  class DeFrame;

  //Typedefs to remove compile errors on windoze.
  typedef double deFloat;

  typedef PrVector3 deVector3;
  typedef PrVector3 SAIVector3;
  typedef PrMatrix3 SAIMatrix3;
  typedef DeDNode taoDNode;
  typedef DeNode taoNode;
  typedef DeFrame deFrame;
  //NOTE TODO : Is there any specific root node for arachi?
  typedef DeBaseNode taoNodeRoot;
#endif

#ifndef ROBARCH_IS_WINDOWS_
  #include <saimatrix/SAIVector.h>
  #include <saimatrix/SAIMatrix.h>

  #include <tao/matrix/TaoDeMath.h>
  #include <tao/dynamics/taoNode.h>
  #include <tao/dynamics/taoDynamics.h>
  #include <tao/utility/TaoDeMassProp.h>
  #include <tao/dynamics/taoJoint.h> 

  using namespace std;

  //Defined in the headers
  class taoDNode;
  class taoNode;
  class taoNodeRoot;
#endif

namespace robotarchitect
{
							 
/**This structure contains all the information required to construct
 * a robot link. Each robot is completely defined by a tree of such 
 * links. */
struct SControllerRobotLink
{
public:    
	//Constructor@End of the class:
	
	//***********************
  //Link-specific data:
  
  //1. Identifiers
  bool is_root;  
	int link_id, parent_link_id;  
  string linkName_;
  string parentName_;
  string robotName_;
  string jointName_;
  
  //2. Tree structure information:
  SControllerRobotLink* parentAddr;
  vector<SControllerRobotLink*> childAddrVector;
  
  
  //***********************
  //TAO-link specific data:
  
  //1. Tao tree structure information:
  taoNode* taoNodeAddr; //This node in the tao tree
  taoNodeRoot* taoNodeRootAddr; //The root of the tao tree
  
  //2. TAO Physical Properties
  deVector3 com_; //Center of mass  
  int jointAxis_; //[x=0 y=1 z=2]
  deFloat mass_;
  deVector3 inertia_;
  deVector3 rotAxis_;
  deFloat rotAngle_;
  deFrame homeFrame_;
  int linkIsFixed_;
  JointType jointType_;
  bool jointIsFree_; //Is the joint controlled or not
  deFloat joint_limit_lower, joint_limit_upper;
  
  //Other useful joint information.
  deFloat joint_default_pos_;
  
	SControllerRobotLink()
	{
		is_root = false;  
		link_id = -2; //arbitrary  
		parent_link_id = -2; //arbitrary
	  linkName_ = "not_assigned";
	  parentName_ = "not_assigned";
	  robotName_ = "not_assigned";
	  jointName_ = "not_assigned";
	  joint_limit_lower = -3.14;
	  joint_limit_upper = 3.14; 
	  
	  //Tree structure information:
	  parentAddr = NULL;  
	  childAddrVector.clear();
	  
	  //TAO Physical Properties
		taoNodeAddr = NULL;
		taoNodeRootAddr = NULL;
		
		//Link's Physical Properties
#ifndef ROBARCH_IS_WINDOWS_
	  com_.set(0, 0, 0);
	  inertia_.set(0,0,0);
	  rotAxis_.set(0,0,0);
#endif
	  jointAxis_ = -1;
	  mass_ = -1;	  
	  rotAngle_ = 0;
	  linkIsFixed_ = 0;
	  jointType_ = JT_NOTASSIGNED;
	  jointIsFree_ = false;
	}
};

}//end of namespace robotarchitect

#endif /*ROBOTLINKSTRUCTURES_HPP_*/


/*
  Old comment : Moved to end for clarity.
  //NOTE TODO Why do we have SAIVector and deVector? Can't we stick to one?
  
//  // Attributes 
//  taoNodeRoot* rootNode_; 
//  list<LinkTag> hasLinkTagVec_;
//  map<int, taoDNode*> idToNodeMap_;
//  map<LinkTag, taoDNode*> linkTagToNodeMap_;
//  map<LinkTag, int> linkTagToIDMap_;
//  map<int, LinkTag> IDToLinkTagMap_;
//  map<JointTag, taoDNode*> jointTagToNodeMap_;
//  map<JointTag, int> jointTagToIDMap_;
//  map<LinkTag, SAIVector> linkTagToSensorMap_;
//NOTE TODO maybe useful?  map<LinkTag, double> linkTagToSurfaceDepthMap_;
//NOTE TODO maybe useful?  int numJoints_;
//  double totalMass_;
//NOTE TODO maybe useful?  SAIVector grav_;
//NOTE TODO maybe useful?  SAIVector forceSensor_; // relative position wrt joint
//  SAIMatrix unactuationMatrix_;
//
//  // Constant
//  static const SAIVector zero3;
//  
//  // Supporting attributes
//  deVector3 pos_;

//  LinkTag linkTag_;
//  JointTag jointTag_;
//  int nodeID_;
//  deFloat defaultJointPos_;
//  deFloat upperJointLimit_;
//  deFloat lowerJointLimit_;
//  map<int, deFloat> defaultJointPosMap_;
//  map<int, deFloat> upperJointLimitMap_;
//  map<int, deFloat> lowerJointLimitMap_;
//  SAIVector defaultJointPosVec_;
//  SAIVector upperJointLimitVec_; //hard jointLimits
//  SAIVector lowerJointLimitVec_; //hard jointLimits
//  int parentNodeID_;

//	//NOTE TODO Fill in the data
*/
