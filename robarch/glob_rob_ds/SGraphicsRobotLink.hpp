
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


namespace robotarchitect
{

/**This structure contains all the information required to construct
 * a robot link. Each robot is completely defined by a tree of such 
 * links. 
 * Also contains graphics data to render the robot. */
struct SGraphicsRobotLink
{
public:
	//NOTE TODO Fill in the data
	//Identifiers
  bool is_root;
  std::string name;
  std::string parent_name;
};


}//end of namespace robotarchitect

#endif /*SARACHIROBOTLINK_HPP_*/
