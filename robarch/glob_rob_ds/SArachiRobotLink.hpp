
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

#ifndef SARACHIROBOTLINK_HPP_
#define SARACHIROBOTLINK_HPP_

//NOTE TODO Substitute these with the appropriate headers after this is added to the build system
#include <map>
#include <list>
#include <string>
#include <vector>

#include "SGlobalRobotDS.hpp"

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

namespace robotarchitect
{
/**This structure contains all the information required to construct
 * a robot link. Each robot is completely defined by a tree of such 
 * links. 
 * Also contains dynamics engine data. */
struct SArachiRobotLink
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
  SArachiRobotLink* parentAddr;
  vector<SArachiRobotLink*> childAddrVector;


  //***********************
  //1. De tree structure information:
  DeNode* deNodeAddr; //This node in the arachi tree
  DeBaseNode* deBaseNodeAddr; //The root of the arachi tree

  //2. De Physical Properties
  PrVector3 com_; //Center of mass  
  int jointAxis_; //[x=0 y=1 z=2]
  wbcFloat mass_;
  PrVector3 inertia_;
  PrVector3 rotAxis_;
  wbcFloat rotAngle_;
  DeFrame homeFrame_;

  int linkIsFixed_;
  JointType jointType_;
  bool jointIsFree_; //Is the joint controlled or not
  wbcFloat joint_limit_lower, joint_limit_upper;

  SArachiRobotLink()
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
    deNodeAddr = NULL;
    deBaseNodeAddr = NULL;

    //Link's Physical Properties
    /*com_.set(0, 0, 0);
  inertia_.set(0,0,0);
  rotAxis_.set(0,0,0);*/

    jointAxis_ = -1;
    mass_ = -1;   
    rotAngle_ = 0;
    linkIsFixed_ = 0;
    jointType_ = JT_NOTASSIGNED;
    jointIsFree_ = false;
  }
};

}//end of namespace robotarchitect

#endif /*SARACHIROBOTLINK_HPP_*/
