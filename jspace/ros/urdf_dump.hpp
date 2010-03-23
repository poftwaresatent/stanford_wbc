/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
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
   \file urdf_dump.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_ROS_URDF_DUMP_HPP
#define JSPACE_ROS_URDF_DUMP_HPP

#include <iosfwd>

namespace urdf {

  class Vector3;
  class Rotation;
  class Pose;
  class Inertial;
  class Link;
  class Model;

}

namespace jspace {
  namespace ros {

  void dump_urdf_tree(std::ostream & os, urdf::Link const & urdf_root, std::string prefix, bool detailed);
  void dump_urdf_tree(std::ostream & os, urdf::Model const & urdf, std::string prefix, bool detailed);

  }
}

namespace std {

  ostream & operator << (ostream & os, urdf::Vector3 const & rhs);
  ostream & operator << (ostream & os, urdf::Rotation const & rhs);
  ostream & operator << (ostream & os, urdf::Pose const & rhs);
  ostream & operator << (ostream & os, urdf::Inertial const & rhs);

}

#endif // JSPACE_ROS_URDF_DUMP_HPP
