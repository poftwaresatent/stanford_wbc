/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2009 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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
   \file tao_dump.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_UTIL_DUMP_HPP
#define JSPACE_UTIL_DUMP_HPP

#include <iosfwd>
#include <vector>
#include <stdexcept>

class deVector6;
class deVector3;
class deQuaternion;
class deFrame;
class taoJoint;
class deMassProp;
class deMatrix3;
class taoDNode;


namespace jspace {
  
  struct tao_tree_info_s;
  
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
  
  /**
     Similar to dump_tao_tree() but uses the more recent tao_tree_info_s structure.
  */
  void dump_tao_tree_info(std::ostream & os, tao_tree_info_s * tree, std::string prefix, bool detailed);
  
  /**
     Similar to dump_tao_tree_info(), but attempts to spew it out in a
     format that can be read back into a SAI XML parser.
  */
  void dump_tao_tree_info_saixml(std::ostream & os, tao_tree_info_s * tree) throw(std::runtime_error);
  
  /**
     Similar to dump_tao_tree_info(), but attempts to spew it out in a
     format that can be read back into a Lotus XML parser.
  */
  void dump_tao_tree_info_lotusxml(std::ostream & os,
				   std::string const & robot_name,
				   std::string const & root_link_name,
				   tao_tree_info_s * tree) throw(std::runtime_error);
  
}

namespace std {
  
  ostream & operator << (ostream & os, deVector6 const & vec);
  ostream & operator << (ostream & os, deVector3 const & vec);
  ostream & operator << (ostream & os, deQuaternion const & vec);
  ostream & operator << (ostream & os, deFrame const & frame);
  ostream & operator << (ostream & os, taoJoint /*const*/ & joint);
  ostream & operator << (ostream & os, deMassProp const & rhs);
  
}

#endif // JSPACE_UTIL_DUMP_HPP
