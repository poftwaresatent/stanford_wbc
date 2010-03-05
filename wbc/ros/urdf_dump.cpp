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
   \file urdf_dump.cpp
   \author Roland Philippsen
*/

#include "urdf_dump.hpp"
#include <urdf/model.h>
#include <iostream>


namespace wbcros {
  
  void dump_urdf_tree(std::ostream & os, urdf::Link const & urdf_root, std::string prefix, bool detailed)
  {
    os << prefix << "* " << urdf_root.name << "\n";
    urdf::Joint const * urdf_joint(urdf_root.parent_joint.get());
    if ( ! urdf_joint) {
      os << prefix << "    no joint\n";
    }
    else {

      os << prefix << "    joint " << urdf_joint->name << ": ";
      switch (urdf_joint->type) {
      case urdf::Joint::UNKNOWN:    os << "UNKNOWN\n"; break;
      case urdf::Joint::REVOLUTE:   os << "REVOLUTE\n"; break;
      case urdf::Joint::CONTINUOUS: os << "CONTINUOUS\n"; break;
      case urdf::Joint::PRISMATIC:  os << "PRISMATIC\n"; break;
      case urdf::Joint::FLOATING:   os << "FLOATING\n"; break;
      case urdf::Joint::PLANAR:     os << "PLANAR\n"; break;
      case urdf::Joint::FIXED:      os << "FIXED\n"; break;
      default:                      os << "<invalid:" << urdf_joint->type << ">\n";
      }
      if (detailed) {
	os << prefix << "      home frame: " << urdf_joint->parent_to_joint_origin_transform << "\n"
	   << prefix << "      axis:       " << urdf_joint->axis << "\n";
      }
    }
    
    if (detailed) {
      if ( ! urdf_root.inertial) {
	os << prefix << "    no inertia\n";
      }
      else {
	os << prefix << "    inertia: " << *urdf_root.inertial << "\n";
      }
    }
    
    prefix += "  ";
    for (size_t ii(0); ii < urdf_root.child_links.size(); ++ii) {
      dump_urdf_tree(os, *urdf_root.child_links[ii], prefix, detailed);
    }
  }
  
  
  void dump_urdf_tree(std::ostream & os, urdf::Model const & urdf, std::string prefix, bool detailed)
  {
    urdf::Link const & root(*urdf.getRoot());
    dump_urdf_tree(os, root, prefix, detailed);
  }
  
}

namespace std {
  
  ostream & operator << (ostream & os, urdf::Vector3 const & rhs)
  {
    os << "{ " << rhs.x << "  " << rhs.y << "  " << rhs.z << " }";
    return os;
  }
  

  ostream & operator << (ostream & os, urdf::Rotation const & rhs)
  {
    os << "{ " << rhs.x << "  " << rhs.y << "  " << rhs.z << "  " << rhs.w << " }";
    return os;
  }
  

  ostream & operator << (ostream & os, urdf::Pose const & rhs)
  {
    os << "t: " << rhs.position << "  r: " << rhs.rotation;
    return os;
  }
  

  ostream & operator << (ostream & os, urdf::Inertial const & rhs)
  {
    os << "m: " << rhs.mass << "  COM: " << rhs.origin << "  I: { XX: " << rhs.ixx << "  xy: " << rhs.ixy
       << "  xz: " << rhs.ixz << "  YY: " << rhs.iyy << "  yz: " << rhs.iyz << "  ZZ: " << rhs.izz << " }";
    return os;
  }

}
