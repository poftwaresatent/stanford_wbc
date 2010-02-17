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

#ifndef SGLOBALROBOTDS_HPP_
#define SGLOBALROBOTDS_HPP_

#include <string>

namespace wbc_tinyxml {
class TiXmlElement;
}

namespace robotarchitect
{

typedef float wbcFloat;

//NOTE TODO we could consider removing these enums:
typedef enum{
  Null_Joint_Tag, Shoulder_Yaw, Shoulder_Pitch,
  Shoulder_Roll, Elbow, Wrist_Roll1,
  Wrist_Pitch, Wrist_Roll2 } JointTag;

typedef enum{
  Null_Link_Tag, Upper_Arm, Lower_Arm, End_Effector,
	Right_Hand, Left_Hand , Hip, Right_Foot, Left_Foot} LinkTag;

typedef enum {
	JT_PRISMATIC = 0, JT_REVOLUTE = 1, JT_SPHERICAL = 2,
	JT_MAX = 3, JT_NOTASSIGNED = -1}JointType;

struct SCameraDS
{
  wbcFloat pos_[3];
  wbcFloat lookat_[3];
  wbcFloat up_[3];
  std::string name_;
};

/**This structure contains all the non-robot specification
 * information required to construct a robotic world.
 * Individual robot definitions are required in addition to this
 * in order to create a robotic simulation environment with controllable
 * robots.
 */
struct SGlobalRobotDS
{
public:
	//Constructor@End of the class:

	//***********************
  //Global data:
	wbcFloat gravity_[3];

  SCameraDS camera_;
};
}


#endif //SGLOBALROBOTDS_HPP_
