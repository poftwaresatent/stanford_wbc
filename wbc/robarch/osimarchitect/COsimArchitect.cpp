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
#include <wbcnet/log.hpp>
static wbcnet::logger_t logger(wbcnet::get_logger("robotarchitect"));

namespace robotarchitect {

COsimArchitect::COsimArchitect() {
}

COsimArchitect::~COsimArchitect() {
}

bool COsimArchitect::readRobotDefinition(const string arg_file) {
  bool flag = true;
  
  //Read osim file:
  osimModel.loadOsimFile(arg_file.c_str());

   #ifndef ROBARCH_IS_WINDOWS_
    robdef_ = new CRobotDefinition<SControllerRobotLink> (true);

    vector<CSkeletonLinkNew>::const_iterator skeletonLink, skeletonLinkE;
    SControllerRobotLink tLnk; //Temporary link

    deVector3 tVec;
    deQuaternion tQuat;
    // 1. First add the global data
    //sai xyz = osim x-zy (Euler rotated base about x by pi/2)
    robdef_->globalRobData_.gravity_[0] = (const wbcFloat) osimModel.gravity.dataPtr()[0];
    robdef_->globalRobData_.gravity_[1] = (const wbcFloat) osimModel.gravity.dataPtr()[1];
    robdef_->globalRobData_.gravity_[2] = (const wbcFloat) osimModel.gravity.dataPtr()[2];

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

      tLnk.link_name_ = (skeletonLink)->name;
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
    	(skeletonLink)->parent_id == -1 ? tLnk.is_root_ = true : tLnk.is_root_= false; //root link check
      tLnk.parent_link_name_ = "rootLink_NoParent";
        tLnk.joint_name_ = "rootLink_NoJoint";
        //sai xyz = osim x-zy (Euler rotated base about x by pi/2)
        tLnk.rot_axis_.set(1, 0, 0);
        tLnk.rot_angle_ = 1.57079633;
      }
      else {
    	tLnk.is_root_= false;
        tLnk.parent_link_name_ = (*skeleJoint).parent_body; //Parent link's name
        tLnk.joint_name_ = (*skeleJoint).name; //The name of the joint between the link and its parent

        (*skeleJoint).location_in_parent.getValues(tVec, 3); //Get the link's position wrt parent

        //Convert axis-angle rotation to quaternions for the link's rotation wrt its parent
        //axis-angle rotation
        tLnk.rot_axis_.set((*skeleJoint).orientation_in_parent_axisangle.x,
            (*skeleJoint).orientation_in_parent_axisangle.y,
            (*skeleJoint).orientation_in_parent_axisangle.z);
        tLnk.rot_angle_ = (*skeleJoint).orientation_in_parent_axisangle.theta;

        tQuat.set(tLnk.rot_axis_, tLnk.rot_angle_);//Compute quaternions for axis-angle
        tLnk.home_frame_.set(tQuat, tVec); //Set the home position and rotation.

        tLnk.joint_type_ = JT_REVOLUTE; // NOTE TODO Set to actual joint type
        tLnk.joint_axis_ = (*skeleJoint).rotates_about_;

        if((*skeleJoint).translates_about_!=-1)
        {
          tLnk.joint_type_ = JT_PRISMATIC; // NOTE TODO Set to actual joint type
          tLnk.joint_axis_ = (*skeleJoint).translates_about_;
        }

        tLnk.joint_default_pos_ = (*skeleJoint).default_pos_;
      }

      /*********************
       * ADD LINK TO POOL
       ********************/
      LOG_INFO (logger, "Adding link:"<< tLnk.link_name_<<", Parent:"<<tLnk.parent_link_name_);
      flag = flag && addLink(tLnk); //If the flag is false once, it remains false.
    }//End of for loop over all links
   #endif
  
  //CRobotDefinition<SControllerRobotLink>* robdef_
  return flag;
}

}
