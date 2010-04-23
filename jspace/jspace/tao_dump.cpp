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
   \file tao_dump.cpp
   \author Roland Philippsen
*/

#include "tao_dump.hpp"
#include "tao_util.hpp"

#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoVar.h>
#include <tao/utility/TaoDeMassProp.h>
#include <iostream>
#include <sstream>


static void dump_deFloat(std::ostream & os, deFloat const * arr, size_t len)
{
  if (0 == arr)
    os << "<NULL>";
  else {
    os << "{ ";
    for (size_t ii(0); ii < len; ++ii) {
      if (ii > 0)
	os << "  ";
      os << arr[ii];
    }
    os << " }";
  }
}

namespace jspace {

std::string inertia_matrix_to_string(deMatrix3 const & mx)
{
  if (0 == &mx) {
    return "<NULL>";
  }
  
  // WARNING: do not just directly access the underlying array of
  // deFloat... because deMatrix3 actually implements a 4x4 matrix for
  // whatever reason they had back when.
  std::ostringstream os;
  os << "{ XX: " << mx.elementAt(0, 0)
     << "  xy: " << mx.elementAt(0, 1)
     << "  xz: " << mx.elementAt(0, 2)
     << "  YY: " << mx.elementAt(1, 1)
     << "  yz: " << mx.elementAt(1, 2)
     << "  ZZ: " << mx.elementAt(2, 2)
     << " }";
  std::string result(os.str());
  return result;
}

  
  void dump_tao_tree(std::ostream & os, taoDNode * root, std::string prefix,
		     bool detailed,
		     std::vector<std::string> * id_to_link_name,
		     std::vector<std::string> * id_to_joint_name)
  {
    if ((0 <= root->getID()) && id_to_link_name && (id_to_link_name->size() > static_cast<size_t>(root->getID()))) {
      os << prefix << "* " << (*id_to_link_name)[root->getID()]
	 << " (ID " << root->getID() << " at "<< (void*) root << ")\n";
    }
    else {
      os << prefix << "* ID " << root->getID() << " at "<< (void*) root << "\n";
    }
    
    os << prefix << "    home:         " << *root->frameHome() << "\n"
       << prefix << "    center:       " << *root->center() << "\n"
       << prefix << "    mass:         " << *root->mass() << "\n"
       << prefix << "    inertia:      " << inertia_matrix_to_string(*root->inertia()) << "\n";
    if (id_to_joint_name && (id_to_joint_name->size() > static_cast<size_t>(root->getID()))) {
      os << prefix << "    joint name:   " << (*id_to_joint_name)[root->getID()] << "\n";
    }
    for (taoJoint /*const*/ * jlist(root->getJointList()); jlist != 0; jlist = jlist->getNext()) {
      os << prefix << "    joint:        " << *jlist << "\n";
    }
    
    if (detailed) {
      os << prefix << "    velocity:     " << *root->velocity() << "\n"
	 << prefix << "    acceleration: " << *root->acceleration() << "\n"
	 << prefix << "    force:        " << *root->force() << "\n"
	 << prefix << "    local:        " << *root->frameLocal() << "\n"
	 << prefix << "    global:       " << *root->frameGlobal() << "\n";
    }
    
    prefix += "  ";
    for (taoDNode * child(root->getDChild()); child != 0; child = child->getDSibling())
      dump_tao_tree(os, child, prefix, detailed, id_to_link_name, id_to_joint_name);
  }
  
  
  static void _dump_tao_tree_info(std::ostream & os, taoDNode * root, tao_tree_info_s::node_info_t const & info,
				  std::string prefix, bool detailed)
  {
    int const id(root->getID());
    if ((0 <= id) && (info.size() > static_cast<size_t>(id))) {
      os << prefix << "* " << info[id].link_name
	 << " (ID " << id << " at "<< (void*) root << ")\n";
    }
    else {
      os << prefix << "* ID " << id << " at "<< (void*) root << "\n";
    }
    
    os << prefix << "    home:         " << *root->frameHome() << "\n"
       << prefix << "    center:       " << *root->center() << "\n"
       << prefix << "    mass:         " << *root->mass() << "\n"
       << prefix << "    inertia:      " << inertia_matrix_to_string(*root->inertia()) << "\n";
    if (info.size() > static_cast<size_t>(id)) {
      os << prefix << "    joint name:   " << info[id].joint_name << "\n"
	 << prefix << "    lower limit:  " << info[id].limit_lower << "\n"
	 << prefix << "    upper limit:  " << info[id].limit_upper << "\n";
    }
    for (taoJoint /*const*/ * jlist(root->getJointList()); jlist != 0; jlist = jlist->getNext()) {
      os << prefix << "    joint:        " << *jlist << "\n";
    }
    
    if (detailed) {
      os << prefix << "    velocity:     " << *root->velocity() << "\n"
	 << prefix << "    acceleration: " << *root->acceleration() << "\n"
	 << prefix << "    force:        " << *root->force() << "\n"
	 << prefix << "    local:        " << *root->frameLocal() << "\n"
	 << prefix << "    global:       " << *root->frameGlobal() << "\n";
    }
    
    prefix += "  ";
    for (taoDNode * child(root->getDChild()); child != 0; child = child->getDSibling())
      _dump_tao_tree_info(os, child, info, prefix, detailed);
  }
  
  
  void dump_tao_tree_info(std::ostream & os, tao_tree_info_s * tree, std::string prefix, bool detailed)
  {
    _dump_tao_tree_info(os, tree->root, tree->info, prefix, detailed);
  }
  
  
  static void _xmldump_tao_tree_info(std::ostream & os, taoDNode * root, tao_tree_info_s::node_info_t const & info,
				     std::string prefix) throw(std::runtime_error)
  {
    deFrame * const home(root->frameHome());
    if ( ! home) {
	throw std::runtime_error("_xmldump_tao_tree_info(): no home frame");
    }
    deVector3 const & pos(home->translation());
    deVector3 raxis;
    deFloat rangle;
    home->rotation().get(raxis, rangle);
    
    int const id(root->getID());
    std::string closing;
    if (0 > id) {
      os << prefix << "<baseNode>\n"
	 << prefix << "  <gravity>0, 0, -9.81</gravity>\n"
	 << prefix << "  <pos>" << pos[0] << ", " << pos[1] << ", " << pos[2] << "</pos>\n"
	 << prefix << "  <rot>" << raxis[0] << ", " << raxis[1] << ", " << raxis[2] <<  ", " << rangle << "</rot>\n";
      closing = prefix + "</baseNode>\n";
    }
    else {
      deVector3 const & com(*root->center());
      deMatrix3 const * inertia(root->inertia());
      taoJoint const * joint(root->getJointList());
      if ( ! joint) {
	throw std::runtime_error("_xmldump_tao_tree_info(): no joint");
      }
      std::string jtype;
      double mixx(0);
      double miyy(0);
      double mizz(0);
      if (0 != dynamic_cast<taoJointPrismatic const *>(joint)) {
	jtype = "P";
	
      }
      else if (0 != dynamic_cast<taoJointRevolute const *>(joint)) {
	jtype = "R";
	mixx = (pow(com[1], 2) + pow(com[2], 2)) * *(root->mass());
	miyy = (pow(com[0], 2) + pow(com[2], 2)) * *(root->mass());
	mizz = (pow(com[0], 2) + pow(com[1], 2)) * *(root->mass());
      }
      else {
	throw std::runtime_error("_xmldump_tao_tree_info(): invalid joint type");
      }
      std::string jaxis;
      taoJointDOF1 const * jdof1(dynamic_cast<taoJointDOF1 const *>(joint));
      if (TAO_AXIS_X == jdof1->getAxis()) {
	jaxis = "X";
      }
      else if (TAO_AXIS_Y == jdof1->getAxis()) {
	jaxis = "Y";
      }
      else if (TAO_AXIS_Z == jdof1->getAxis()) {
	jaxis = "Z";
      }
      else {
	throw std::runtime_error("_xmldump_tao_tree_info(): invalid joint axis");
      }
      
      os << prefix << "<jointNode>\n"
	 << prefix << "  <ID>" << id << "</ID>\n"
	 << prefix << "  <type>" << jtype << "</type>\n"
	 << prefix << "  <axis>" << jaxis << "</axis>\n"
	 << prefix << "  <mass>" << *(root->mass()) << "</mass>\n"
	 << prefix << "  <inertia>" << inertia->elementAt(0, 0) - mixx
	 << ", " << inertia->elementAt(1, 1) - miyy
	 << ", " << inertia->elementAt(2, 2) - mizz << "</inertia>\n"
	 << prefix << "  <com>" << com[0] << ", " << com[1] << ", " << com[2] << "</com>\n"
	 << prefix << "  <pos>" << pos[0] << ", " << pos[1] << ", " << pos[2] << "</pos>\n"
	 << prefix << "  <rot>" << raxis[0] << ", " << raxis[1] << ", " << raxis[2] <<  ", " << rangle << "</rot>\n";
      closing = prefix + "</jointNode>\n";
    }
    
    prefix += "  ";
    for (taoDNode * child(root->getDChild()); child != 0; child = child->getDSibling())
      _xmldump_tao_tree_info(os, child, info, prefix);
    
    os << closing;
  }
  
  
  void xmldump_tao_tree_info(std::ostream & os, tao_tree_info_s * tree) throw(std::runtime_error)
  {
    os << "<?xml version=\"1.0\" ?>\n"
       << "<dynworld>\n";
    std::string prefix("  ");
    _xmldump_tao_tree_info(os, tree->root, tree->info, prefix);
    os << "</dynworld>\n";
  }
  
}

namespace std {
  

  ostream & operator << (ostream & os, deVector6 const & vec) {
    if (0 == &vec)
      os << "<NULL>";
    else
      dump_deFloat(os, &vec.elementAt(0), 6);
    return os;
  }
  

  ostream & operator << (ostream & os, deVector3 const & vec) {
    if (0 == &vec)
      os << "<NULL>";
    else
      dump_deFloat(os, &vec.elementAt(0), 3);
    return os;
  }
  

  ostream & operator << (ostream & os, deQuaternion const & vec) {
    if (0 == &vec)
      os << "<NULL>";
    else
      dump_deFloat(os, &vec[0], 4);
    return os;
  }
  

  ostream & operator << (ostream & os, deFrame const & frame) {
    if (0 == &frame)
      os << "<NULL>";
    else
      os << "r: " << frame.rotation() << "  t: " << frame.translation();
    return os;
  }


  ostream & operator << (ostream & os, taoJoint /*const*/ & joint) {
    if (0 == &joint) {
      os << "<NULL>";
      return os;
    }
    taoJointType const jtype(joint.getType());
    switch (jtype) {
    case TAO_JOINT_PRISMATIC:
      os << "prismatic (axis " << dynamic_cast<taoJointDOF1 /*const*/ *>(&joint)->getAxis() << ")";
      break;
    case TAO_JOINT_REVOLUTE:
      os << "revolute (axis " << dynamic_cast<taoJointDOF1 /*const*/ *>(&joint)->getAxis() << ")";
      break;
    case TAO_JOINT_SPHERICAL: os << "spherical "; break;
    case TAO_JOINT_USER:      os << "user "; break;
    default:                  os << "<invalid type: " << jtype << "> ";
    }
    os << "  " << joint.getDOF() << " DOF";
    std::vector<deFloat> foo(joint.getDOF());
    joint.getQ(&foo[0]);
    os << "  q: ";
    dump_deFloat(os, &foo[0], joint.getDOF());
    joint.getDQ(&foo[0]);
    os << "  dq: ";
    dump_deFloat(os, &foo[0], joint.getDOF());
    joint.getDDQ(&foo[0]);
    os << "  ddq: ";
    dump_deFloat(os, &foo[0], joint.getDOF());
    joint.getTau(&foo[0]);
    os << "  tau: ";
    dump_deFloat(os, &foo[0], joint.getDOF());
    return os;
  }
  

  ostream & operator << (ostream & os, deMassProp const & rhs)
  {
    deFloat mass;
    deVector3 center;
    deMatrix3 inertia;
    rhs.get(&mass, &center, &inertia);
    os << "m: " << mass << "  COM: " << center
       << "  I: " << jspace::inertia_matrix_to_string(inertia);
    return os;
  }
  
}
