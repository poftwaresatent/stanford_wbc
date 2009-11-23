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
//=========================================================================
/*!
 \author     Samir Menon
 \file       COsimArchitect.cpp (opensim file parser)
 */
//=========================================================================
#ifdef TESTING_FUNCTIONS_ON
#include <iostream>
#endif 

#include "COsimArchitect.hpp"
#ifndef ROBARCH_IS_WINDOWS_
  #include <wbcnet/log.hpp>
  static wbcnet::logger_t logger(wbcnet::get_logger("robotarchitect"));
#endif

namespace robotarchitect {

COsimArchitect::COsimArchitect() {
}

COsimArchitect::~COsimArchitect() {
}

bool COsimArchitect::readRobotDefinition(const string arg_file,
    const bool buildControllerRobot, const bool buildGraphicsRobot) {
  bool flag = true;
  
  buildCrRobot = buildControllerRobot; //Set own flags
  buildGrRobot = buildGraphicsRobot; //Set own flags

  if (!(buildControllerRobot || buildGraphicsRobot)) {//No robot requested.
    return false;
  }

  //Read osim file:
  osimModel.loadOsimFile(arg_file.c_str());

  if (buildControllerRobot) {
   #ifndef ROBARCH_IS_WINDOWS_
    crRobotDef = new CRobotDefinition<SControllerRobotLink> (true);

    vector<CSkeletonLinkNew>::const_iterator skeletonLink, skeletonLinkE;
    SControllerRobotLink tLnk; //Temporary link

    deVector3 tVec;
    deQuaternion tQuat;
    // 1. First add the global data
    //sai xyz = osim x-zy (Euler rotated base about x by pi/2)
    crRobotDef->globalRobData_.gravity_[0] = (const wbcFloat) osimModel.gravity.dataPtr()[0];
    crRobotDef->globalRobData_.gravity_[1] = (const wbcFloat) osimModel.gravity.dataPtr()[1];
    crRobotDef->globalRobData_.gravity_[2] = (const wbcFloat) osimModel.gravity.dataPtr()[2];

    // 2. Load, build all links
    for (skeletonLink = osimModel.vSimmBodyVector.begin(), skeletonLinkE
        = osimModel.vSimmBodyVector.end(); skeletonLink != skeletonLinkE; ++skeletonLink) {
      /*********************
       * LINK DATA
       ********************/
      tLnk.mass_ = (skeletonLink)->mass; // mass property
      tLnk.inertia_.set((skeletonLink)->inertia_xx, (skeletonLink)->inertia_yy,
          (skeletonLink)->inertia_zz); // inertia property
      tLnk.com_.set((skeletonLink)->mass_center[0],
          (skeletonLink)->mass_center[1], (skeletonLink)->mass_center[2]);// center of mass property

      tLnk.linkName_ = (skeletonLink)->name;
      //		  string robotName_;//NOTE TODO Fill this in with the name of the root


      /*********************
       * JOINT DATA
       ********************/
      //NOTE Link = non-base node. Each link has a joint with its parent link.
      /*NOTE TODO In osim files each joint could have multiple dof! Each dof is stored in a coordinate.
       *Presently we assume that each joint has one link. */
      vector<CJointNew>::const_iterator skeleJoint =
          (skeletonLink)->jointVector.begin(); //Single joint!

      if (skeleJoint == (skeletonLink)->jointVector.end()) // Root node -- Contains no joints
      {
    	(skeletonLink)->parent_id == -1 ? tLnk.is_root = true : tLnk.is_root= false; //root link check
        tLnk.parentName_ = "rootLink_NoParent";
        tLnk.jointName_ = "rootLink_NoJoint";
        //sai xyz = osim x-zy (Euler rotated base about x by pi/2)
        tLnk.rotAxis_.set(1, 0, 0);
        tLnk.rotAngle_ = 1.57079633;
      }
      else {
    	tLnk.is_root= false;
        tLnk.parentName_ = (*skeleJoint).parent_body; //Parent link's name
        tLnk.jointName_ = (*skeleJoint).name; //The name of the joint between the link and its parent

        (*skeleJoint).location_in_parent.getValues(tVec, 3); //Get the link's position wrt parent

        //Convert axis-angle rotation to quaternions for the link's rotation wrt its parent
        //axis-angle rotation
        tLnk.rotAxis_.set((*skeleJoint).orientation_in_parent_axisangle.x,
            (*skeleJoint).orientation_in_parent_axisangle.y,
            (*skeleJoint).orientation_in_parent_axisangle.z);
        tLnk.rotAngle_ = (*skeleJoint).orientation_in_parent_axisangle.theta;

        tQuat.set(tLnk.rotAxis_, tLnk.rotAngle_);//Compute quaternions for axis-angle
        tLnk.homeFrame_.set(tQuat, tVec); //Set the home position and rotation.

        tLnk.jointType_ = JT_REVOLUTE; // NOTE TODO Set to actual joint type
        tLnk.jointAxis_ = (*skeleJoint).rotates_about_;

        if((*skeleJoint).translates_about_!=-1)
        {
          tLnk.jointType_ = JT_PRISMATIC; // NOTE TODO Set to actual joint type
          tLnk.jointAxis_ = (*skeleJoint).translates_about_;
        }
      }

      /*********************
       * ADD LINK TO POOL
       ********************/
      LOG_INFO (logger, "Adding link:"<< tLnk.linkName_<<", Parent:"<<tLnk.parentName_);
      flag = flag && addLink(tLnk); //If the flag is false once, it remains false.
    }//End of for loop over all links
   #endif
  }
  if (buildGraphicsRobot) {
    grRobotDef = new CRobotDefinition<SGraphicsRobotLink> (true);
  }

  //CRobotDefinition<SControllerRobotLink>* crRobotDef
  return flag;
}

}
