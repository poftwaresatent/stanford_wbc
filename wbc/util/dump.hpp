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
   \file dump.hpp
   \author Roland Philippsen
*/

#ifndef WBC_UTIL_DUMP_HPP
#define WBC_UTIL_DUMP_HPP

#include <iosfwd>
#include <vector>

class deVector6;
class deVector3;
class deQuaternion;
class deFrame;
class taoJoint;
class deMassProp;
class deMatrix3;
class taoDNode;

#ifdef HAVE_URDF

namespace urdf {
  class Vector3;
  class Rotation;
  class Pose;
  class Inertial;
  class Link;
}

#endif // HAVE_URDF


namespace wbc {

  std::string inertia_matrix_to_string(deMatrix3 const & mx);
  
  /**
     Write a textual description of the TAO tree rooted at the given
     node to the given std::ostream. Each line of output is prefixed
     by the given \c prefix, and each level of the tree is indented
     two spaces with respect to the previous level. If the \c detailed
     flag is specified, then a lot more information is given for each
     node. Optionally, you can pass a pointer to a vector containing
     the node names, which will then be output along with the ID (if
     there is an entry for that ID in the given \c
     id_to_link_name).
  */
  void dump_tao_tree(std::ostream & os, taoDNode * root, std::string prefix,
		     bool detailed,
		     std::vector<std::string> * id_to_link_name);
  
#ifdef HAVE_URDF
  void dump_urdf_tree(std::ostream & os, urdf::Link const & urdf_root, std::string prefix, bool detailed);
#endif // HAVE_URDF

}

namespace std {
  
  ostream & operator << (ostream & os, deVector6 const & vec);
  ostream & operator << (ostream & os, deVector3 const & vec);
  ostream & operator << (ostream & os, deQuaternion const & vec);
  ostream & operator << (ostream & os, deFrame const & frame);
  ostream & operator << (ostream & os, taoJoint /*const*/ & joint);
  ostream & operator << (ostream & os, deMassProp const & rhs);
  
#ifdef HAVE_URDF

  ostream & operator << (ostream & os, urdf::Vector3 const & rhs);
  ostream & operator << (ostream & os, urdf::Rotation const & rhs);
  ostream & operator << (ostream & os, urdf::Pose const & rhs);
  ostream & operator << (ostream & os, urdf::Inertial const & rhs);

#endif // HAVE_URDF
  
}

#endif // WBC_UTIL_DUMP_HPP
