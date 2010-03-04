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


namespace wbc {

  std::string inertia_matrix_to_string(deMatrix3 const & mx);
  
  /**
     Write a textual description of the TAO tree rooted at the given
     node to the given std::ostream. Each line of output is prefixed
     by the given \c prefix, and each level of the tree is indented
     two spaces with respect to the previous level. If the \c detailed
     flag is specified, then a lot more information is given for each
     node. Optionally, you can pass pointers to vectors containing the
     node and joint names, which will then be output along with the ID
     (if there is an entry for that ID in the given \c
     id_to_link_name). Use \c NULL for \c id_to_link_name and/or \c
     id_to_link_name if you do not have that information, or don't
     care to have it printed.
  */
  void dump_tao_tree(std::ostream & os, taoDNode * root, std::string prefix,
		     bool detailed,
		     std::vector<std::string> * id_to_link_name,
		     std::vector<std::string> * id_to_joint_name);

}

namespace std {
  
  ostream & operator << (ostream & os, deVector6 const & vec);
  ostream & operator << (ostream & os, deVector3 const & vec);
  ostream & operator << (ostream & os, deQuaternion const & vec);
  ostream & operator << (ostream & os, deFrame const & frame);
  ostream & operator << (ostream & os, taoJoint /*const*/ & joint);
  ostream & operator << (ostream & os, deMassProp const & rhs);
  
}

#endif // WBC_UTIL_DUMP_HPP
