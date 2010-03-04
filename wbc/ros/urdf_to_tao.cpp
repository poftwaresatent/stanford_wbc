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
   \file urdf_to_tao.cpp Convert a URDF robot model into a TAO tree.
   \author Roland Philippsen
*/

#include "urdf_to_tao.hpp"
#include <fstream>
#include <limits>

using namespace std;


namespace urdf_to_tao {




  void ActiveLinkFilter::
  AddLink(std::string const & link_name)
  {
    m_active.insert(link_name);
  }
  
  
  void FlatFileLinkFilter::
  Load(std::string const & filename) throw(std::runtime_error)
  {
    ifstream config(filename.c_str());
    if ( ! config) {
      throw runtime_error("urdf_to_tao::FlatFileLinkFilter::Load(" + filename + "): could not open file");
    }
    
    string token;
    while (config >> token) {
      if (token[0] == '#'){
	config.ignore(numeric_limits<streamsize>::max(), '\n');
	continue;
      }
      if (m_root_name.empty()) {
	m_root_name = token;
      }
      AddLink(token);
    }
    
    if (m_root_name.empty()) {
      throw runtime_error("FlatFileLinkFilter::Load(" + filename + "): no specs in file?");
    }
  }
  
  
  std::string const & FlatFileLinkFilter::
  GetRootName() const
  {
    return m_root_name;
  }
  
}


#include "urdf_dump.hpp"
#include <wbc/util/dump.hpp>
#include <wbc/core/BranchingRepresentation.hpp>

// Quick hack around build sys bug: urdf_to_tao depends on ROS, which has log4cxx.
#define HAVE_LOG4CXX
#include <wbcnet/log.hpp>

#include <tao/utility/TaoDeMassProp.h>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <urdf/model.h>
#include <set>
#include <map>
#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("urdf_to_tao"));

using wbc::inertia_matrix_to_string;


namespace urdf_to_tao {
  
  
  bool DefaultLinkFilter::
  isFixed(urdf::Link const & urdf_link) const
  {
    if ( ! urdf_link.parent_joint)
      return false;
    return urdf::Joint::FIXED == urdf_link.parent_joint->type;
  }
  
  
  bool ActiveLinkFilter::
  isFixed(urdf::Link const & urdf_link) const
  {
    if (urdf_to_tao::DefaultLinkFilter::isFixed(urdf_link)) {
      return true;
    }
    if (m_active.find(urdf_link.name) != m_active.end()) {
      return false;
    }
    return true;
  }
  
  
  class element;
  
  typedef std::set<element *> forest_t;
  
  class element
  {
    friend class Converter;
    
    element(element * parent, boost::shared_ptr<urdf::Link const> urdf_link, bool is_fixed);
    element(element * parent, element const & original);
    
  public:
    bool isFixed() const;
    std::string const & getName() const;
    
    element * parent;
    forest_t children;
    bool is_fixed;
    
    boost::shared_ptr<urdf::Link const> const urdf_link;
    
    //     // XXXX start computing and using urdf_aux_frames
    //     typedef std::map<std::string, urdf::Pose> urdf_aux_frames_t;
    //     urdf_aux_frames_t urdf_aux_frames;
    
    deMassProp tao_mass_prop;
    deFrame tao_home_frame;
  };
  
  
  class Converter {
  public:
    explicit Converter(LinkFilter const & _link_filter)
      : link_filter(_link_filter) {}
    
    ~Converter() {
      for (forest_t::iterator ii(all_elements.begin()); ii != all_elements.end(); ++ii)
	delete *ii;
    }
    
    element * create_element(element * parent, boost::shared_ptr<urdf::Link const> urdf_link)
    {
      element * ee(new element(parent, urdf_link, link_filter.isFixed(*urdf_link)));
      all_elements.insert(ee);
      return ee;
    }
    
    element * clone_element(element * parent, element const & original)
    {
      element * ee(new element(parent, original));
      all_elements.insert(ee);
      return ee;
    }
    
    element * read_urdf_tree(element * parent, boost::shared_ptr<urdf::Link const> urdf_child);
    
    element * copy_fuse_fixed(element * parent, element * original);
    
  protected:
    forest_t all_elements;
    LinkFilter const & link_filter;
  };
  
  
  element::
  element(element * _parent,
	  boost::shared_ptr<urdf::Link const> _urdf_link,
	  bool _is_fixed)
    : parent(_parent),
      is_fixed(_is_fixed),
      urdf_link(_urdf_link)
  {
    if (parent)
      parent->children.insert(this);
    
    deFloat mass;
    deVector3 com;
    deMatrix3 inertia;
    urdf::Inertial const * urdf_inertia(_urdf_link->inertial.get());
    if ( ! urdf_inertia) {
      mass = 0;
      com.zero();		// redundant
      inertia.zero();		// redundant
    }
    else {
      mass = urdf_inertia->mass;
      com.set(urdf_inertia->origin.position.x,
	      urdf_inertia->origin.position.y,
	      urdf_inertia->origin.position.z);
      inertia.set(urdf_inertia->ixx, urdf_inertia->ixy, urdf_inertia->ixz,
		  urdf_inertia->ixy, urdf_inertia->iyy, urdf_inertia->iyz,
		  urdf_inertia->ixz, urdf_inertia->iyz, urdf_inertia->izz);
    }
    tao_mass_prop.set(&mass, &com, &inertia);
    
    urdf::Joint const * urdf_joint(_urdf_link->parent_joint.get());
    if ( ! urdf_joint) {
      tao_home_frame.identity(); // redundant
    }
    else {
      urdf::Pose const & urdf_home(urdf_joint->parent_to_joint_origin_transform);
      deVector3 const trans(urdf_home.position.x,
			    urdf_home.position.y,
			    urdf_home.position.z);
      deQuaternion const rot(urdf_home.rotation.x,
			     urdf_home.rotation.y,
			     urdf_home.rotation.z,
			     urdf_home.rotation.w);
      tao_home_frame.set(rot, trans);
    }
  }
  
  
  element::
  element(element * _parent,
	  element const & original)
    : parent(_parent),
      is_fixed(original.is_fixed),
      urdf_link(original.urdf_link),
      tao_mass_prop(original.tao_mass_prop),
      tao_home_frame(original.tao_home_frame)
    {
      if (parent)
	parent->children.insert(this);
    }
  
  
  bool element::
  isFixed() const
  {
    return is_fixed;
  }
  
  
  std::string const & element::
  getName() const
  {
    return urdf_link->name;
  }
  
  
  void dump_tree(std::ostream & os, element const * root, std::string prefix, bool detailed)
  {
    if (root->isFixed())
      os << prefix << "# " << root->getName() << "\n";
    else
      os << prefix << "o " << root->getName() << "\n";
    
    if (detailed) {
      urdf::Inertial const * urdf_inertia(root->urdf_link->inertial.get());
      if ( ! urdf_inertia) {
	os << prefix << "    no urdf_inertia\n";
      }
      else {
	os << prefix << "    urdf_inertia:    " << *urdf_inertia << "\n";
      }
      
      urdf::Joint const * urdf_joint(root->urdf_link->parent_joint.get());
      if ( ! urdf_joint) {
	os << prefix << "    no urdf_home_frame\n";
      }
      else {
	os << prefix << "    urdf_home_frame: " << urdf_joint->parent_to_joint_origin_transform << "\n";
      }
      
      //       // XXXX start computing and using urdf_aux_frames
      //       for (element::urdf_aux_frames_t::const_iterator frame(root->urdf_aux_frames.begin());
      // 	   frame != root->urdf_aux_frames.end(); ++frame)
      // 	os << prefix << "    urdf_aux frame \"" << frame->first << "\"\n"
      // 		  << prefix << "        " << frame->second << "\n";
      
      os << prefix << "    tao_mass_prop:   " << root->tao_mass_prop << "\n";
    }
    
    prefix += "  ";
    for (forest_t::const_iterator child(root->children.begin()); child != root->children.end(); ++child) {
      dump_tree(os, *child, prefix, detailed);
    }
  }
  
  
  element * Converter::
  read_urdf_tree(element * parent,
		 boost::shared_ptr<urdf::Link const> urdf_child)
  {
    element * child(create_element(parent, urdf_child));
    for (size_t ii(0); ii < urdf_child->child_links.size(); ++ii) {
      read_urdf_tree(child, urdf_child->child_links[ii]);
    }
    return child;
  }
  
  
  void inertia_parallel_axis_transform(double in_ixx, double in_ixy, double in_ixz,
				       double in_iyy, double in_iyz, double in_izz,
				       double trans_x, double trans_y, double trans_z, double mass,
				       double & out_ixx, double & out_ixy, double & out_ixz,
				       double & out_iyy, double & out_iyz, double & out_izz)
  {
    double const xx(trans_x * trans_x);
    double const yy(trans_y * trans_y);
    double const zz(trans_z * trans_z);
    out_ixx = in_ixx + mass * (yy + zz);
    out_iyy = in_iyy + mass * (xx + zz);
    out_izz = in_izz + mass * (xx + yy);
    
    out_ixy = in_ixy - mass * (trans_x * trans_y);
    out_ixz = in_ixz - mass * (trans_x * trans_z);
    out_iyz = in_iyz - mass * (trans_y * trans_z);
    
    LOG_TRACE (logger,
	       "             inertia_parallel_axis_transform()\n"
	       << "               XX: "<<in_ixx<<" + "<<mass<<" * ("<<yy<<" + "<<zz<<") = "<<out_ixx<<"\n"
	       << "               YY: "<<in_iyy<<" + "<<mass<<" * ("<<xx<<" + "<<zz<<") = "<<out_iyy<<"\n"
	       << "               ZZ: "<<in_izz<<" + "<<mass<<" * ("<<xx<<" + "<<yy<<") = "<<out_izz<<"\n"
	       << "               xy: "<<in_ixy<<" - "<<mass<<" * ("<<trans_x<<" * "<<trans_y<<") = "<<out_ixy<<"\n"
	       << "               xz: "<<in_ixz<<" - "<<mass<<" * ("<<trans_x<<" * "<<trans_z<<") = "<<out_ixz<<"\n"
	       << "               yz: "<<in_iyz<<" - "<<mass<<" * ("<<trans_y<<" * "<<trans_z<<") = "<<out_iyz);
  }
  
  
  void inertia_parallel_axis_transform(deMatrix3 const & in_inertia,
				       deVector3 const & translation, double mass,
				       deMatrix3 & out_inertia)
  {
    inertia_parallel_axis_transform(in_inertia.elementAt(0, 0),
				    in_inertia.elementAt(0, 1),
				    in_inertia.elementAt(0, 2),
				    in_inertia.elementAt(1, 1),
				    in_inertia.elementAt(1, 2),
				    in_inertia.elementAt(2, 2),
				    translation[0],
				    translation[1],
				    translation[2],
				    mass,
				    out_inertia.elementAt(0, 0),
				    out_inertia.elementAt(0, 1),
				    out_inertia.elementAt(0, 2),
				    out_inertia.elementAt(1, 1),
				    out_inertia.elementAt(1, 2),
				    out_inertia.elementAt(2, 2));
  }
  
  
  void inertia_similarity_transform(double in_ixx, double in_ixy, double in_ixz,
				    double in_iyy, double in_iyz, double in_izz,
				    deQuaternion const & rotation,
				    deMatrix3 & out_inertia)
  {
    out_inertia.set(in_ixx, in_ixy, in_ixz,
		    in_ixy, in_iyy, in_iyz,
		    in_ixz, in_iyz, in_izz);
    deMatrix3 aa, bb, cc;
    aa.set(rotation);
    bb.transpose(aa);
    cc.multiply(aa, out_inertia); // cc = rotation * inertia
    aa.multiply(cc, bb);	// aa = cc * rotation_transpose
    out_inertia += aa;
  }
  
  
  void inertia_similarity_transform(deMatrix3 const & in_inertia,
				    deQuaternion const & rotation,
				    /** OK to have &in_inertia == &out_inertia, we use copies. */
				    deMatrix3 & out_inertia)
  {
    inertia_similarity_transform(in_inertia.elementAt(0, 0), in_inertia.elementAt(0, 1), in_inertia.elementAt(0, 2),
				 in_inertia.elementAt(1, 1), in_inertia.elementAt(1, 2), in_inertia.elementAt(2, 2),
				 rotation, out_inertia);
  }
  
  
  void inertia_similarity_transform(double in_ixx, double in_ixy, double in_ixz,
				    double in_iyy, double in_iyz, double in_izz,
				    double rot_qx, double rot_qy, double rot_qz, double rot_qw,
				    double & out_ixx, double & out_ixy, double & out_ixz,
				    double & out_iyy, double & out_iyz, double & out_izz)
  {
    deQuaternion quat;
    quat.set(rot_qx, rot_qy, rot_qz, rot_qw);
    deMatrix3 out;
    inertia_similarity_transform(in_ixx, in_ixy, in_ixz, in_iyy, in_iyz, in_izz, quat, out);
    out_ixx = out.elementAt(0, 0);
    out_ixy = out.elementAt(0, 1);
    out_ixz = out.elementAt(0, 2);
    out_iyy = out.elementAt(1, 1);
    out_iyz = out.elementAt(1, 2);
    out_izz = out.elementAt(2, 2);
  }
  
  
  /** inertias are expressed wrt to COM */
  void fuse_mass_properties(double orig_mass, deMatrix3 const & orig_inertia, deFrame const & orig_com,
			    double adtl_mass, deMatrix3 const & adtl_inertia, deFrame const & adtl_com,
			    deFrame const & home_of_adtl_wrt_orig,
			    double & fused_mass, deMatrix3 & fused_inertia, deFrame & fused_com)
  {
    LOG_INFO (logger,
	      "      fuse_mass_properties():\n"
	      << "        orig: m = " << orig_mass << "  I = " << inertia_matrix_to_string(orig_inertia)
	      << "  COM = " << orig_com << "\n"
	      << "        adtl: m = " << adtl_mass << "  I = " << inertia_matrix_to_string(adtl_inertia)
	      << "  COM = " << adtl_com << "\n"
	      << "        home_of_adtl_wrt_orig = " << home_of_adtl_wrt_orig);
    
    // 1. compute new total mass
    LOG_DEBUG (logger, "        1. compute new total mass");
    fused_mass = orig_mass + adtl_mass;
    LOG_DEBUG (logger,
	       "           fused_mass = orig_mass + adtl_mass = " << orig_mass << " + " << adtl_mass
	       << " = " << fused_mass);
    
    // 2. compute new COM
    LOG_DEBUG (logger, "        2. compute new COM");
    deFrame adtl_com_wrt_o;
    adtl_com_wrt_o.multiply(home_of_adtl_wrt_orig, deFrame(adtl_com));
    fused_com.translation() = orig_com.translation();
    fused_com.translation() *= orig_mass / fused_mass;
    {
      deVector3 bar;
      bar = adtl_com_wrt_o.translation();
      bar *= adtl_mass / fused_mass;
      fused_com.translation() += bar;
    }
    fused_com.rotation() = orig_com.rotation(); // is this even taken into account anywhere?
    LOG_DEBUG (logger,
	       "           adtl_com_wrt_o = " << adtl_com_wrt_o << "\n"
	       << "           fused_com = " << fused_com.translation());
    
    // 3. parallel axis transform of original inertia to new COM
    LOG_DEBUG (logger, "        3. parallel axis transform of original inertia to new COM");
    deVector3 orig_to_new_com;
    orig_to_new_com = fused_com.translation();
    orig_to_new_com -= orig_com.translation();
    deMatrix3 orig_inertia_at_new_com; // can probably reuse same instance
    inertia_parallel_axis_transform(orig_inertia, orig_to_new_com, orig_mass, orig_inertia_at_new_com);
    LOG_DEBUG (logger,
	       "           orig_to_new_com = " << orig_to_new_com << "\n"
	       << "           orig_inertia_at_new_com = " << inertia_matrix_to_string(orig_inertia_at_new_com));
    
    // 4. similarity transform of additional inertia to original axes
    LOG_DEBUG (logger, "        4. similarity transform of additional inertia to original axes");
    deMatrix3 adtl_inertia_at_new_com;
    inertia_similarity_transform(adtl_inertia,
				 home_of_adtl_wrt_orig.rotation(), // XXXX ignore possible adtl com frame rot
				 adtl_inertia_at_new_com);
    LOG_DEBUG (logger,
	       "           home_of_adtl_wrt_orig = " << home_of_adtl_wrt_orig << "\n"
	       << "           (intermediate) adtl_inertia_at_new_com = "
	       << inertia_matrix_to_string(adtl_inertia_at_new_com));
    
    // 5. parallel axis of additional inertia to new COM (delta COM expressed in original frame)
    //    ---> translate by the delta of old to new COM_adtl in orig frame
    LOG_DEBUG (logger, "        5. parallel axis of additional inertia to new COM");
    deVector3 adtl_to_new_com;
    adtl_to_new_com = adtl_com_wrt_o.translation();
    adtl_to_new_com -= fused_com.translation();
    inertia_parallel_axis_transform(adtl_inertia, adtl_to_new_com, adtl_mass, adtl_inertia_at_new_com);
    LOG_DEBUG (logger,
	       "           adtl_to_new_com = " << adtl_to_new_com << "\n"
	       << "           (final) adtl_inertia_at_new_com = "
	       << inertia_matrix_to_string(adtl_inertia_at_new_com));
    
    // 6. add inertias
    //    ---> add inertia due to the point masses at the COM?
    //    ---> no, masses already taken into account during parallel axis theorem application!
    LOG_DEBUG (logger, "        6. add inertias");
    fused_inertia = orig_inertia_at_new_com;
    fused_inertia += adtl_inertia_at_new_com;
    LOG_INFO (logger, "           fused_inertia = " << inertia_matrix_to_string(fused_inertia));
  }
  
  
  /** inertias are expressed wrt COM */
  void fuse_mass_properties(deMassProp /*const*/ & original,
			    deMassProp /*const*/ & additional,
			    deFrame const & home_of_additional_wrt_original,
			    /** OK to pass same ref as original */
			    deMassProp & fused)
  {
    deFrame orig_com, adtl_com;
    orig_com.translation() = *original.center();
    adtl_com.translation() = *additional.center();
    double fused_mass;
    deMatrix3 fused_inertia;
    deFrame fused_com;
    fuse_mass_properties(*original.mass(), *original.inertia(), orig_com,
			 *additional.mass(), *additional.inertia(), adtl_com,
			 home_of_additional_wrt_original,
			 fused_mass, fused_inertia, fused_com);
    fused.set(&fused_mass, &fused_com.translation(), &fused_inertia);
  }
  
  
  /** inertias are expressed wrt COM */
  element * Converter::
  copy_fuse_fixed(element * parent, element * original)
  {
    // Find all child subtrees that must be fused into the base. Create
    // a new subtree for each one, because in turn they will recursively
    // fuse their fixed children. Those subtrees are temporary objects
    // without parents.
    forest_t fixed_subtrees;
    for (forest_t::const_iterator child(original->children.begin()); child != original->children.end(); ++child) {
      if ((*child)->isFixed()) {
	fixed_subtrees.insert(copy_fuse_fixed(0, *child));
      }
    }
    
    element * fused_duplicate(clone_element(parent, *original));
    {
      //       // XXXX start computing and using urdf_aux_frames
      //       element::urdf_aux_frames_t & urdf_fused_aux(fused_duplicate->urdf_aux_frames);
      
      // Fuse frames and inertias from all subtrees connected via fixed
      // joints. Copy all auxiliary frames, updating them to the new
      // base.
      for (forest_t::const_iterator subtree(fixed_subtrees.begin()); subtree != fixed_subtrees.end(); ++subtree) {
	
	LOG_TRACE (logger,
		   "copy_fuse_fixed():\n"
		   << "  parent:   " << (0 == parent ? "<NULL>" : parent->getName()) << "\n"
		   << "  original: " << original->getName() << "\n"
		   << "  subtree:      " << (*subtree)->getName() << "\n"
		   << "    mass_prop:  " << (*subtree)->tao_mass_prop << "\n"
		   << "    home_frame: " << (*subtree)->tao_home_frame << "\n"
		   << "  inertia fusion:\n"
		   << "    before:     " << fused_duplicate->tao_mass_prop);
	
	fuse_mass_properties(fused_duplicate->tao_mass_prop,
			     (*subtree)->tao_mass_prop,
			     (*subtree)->tao_home_frame,
			     fused_duplicate->tao_mass_prop);
	
	LOG_TRACE (logger, "    after:      " << fused_duplicate->tao_mass_prop);
	
// 	std::string const prefix((*subtree)->getName() + "/");
// 	element::kdl_aux_frames_t const & subtree_aux((*subtree)->kdl_aux_frames);
// 	urdf_fused_aux.insert(make_pair(prefix + "home", urdf_home_frame));
// 	urdf_fused_aux.insert(make_pair(prefix + "tip", urdf_home_frame * (*subtree)->kdl_tip_frame));
// 	for (element::kdl_aux_frames_t::const_iterator iaf(subtree_aux.begin()); iaf != subtree_aux.end(); ++iaf) {
// 	  fused_aux.insert(make_pair(prefix + iaf->first, urdf_home_frame * iaf->second));
// 	}
      }
    }
    
    // Find all non-fixed children, recursively get their fused
    // subtrees, and add them as children to the new base.
    for (forest_t::const_iterator child(original->children.begin()); child != original->children.end(); ++child) {
      if ( ! (*child)->isFixed()) {
	copy_fuse_fixed(fused_duplicate, *child);
      }
    }
    
    // Find all non-fixed children of fixed subtress, and change their
    // ancestry to descend directly from the freshly created fused
    // duplicate. Change their HOME frames according to their new parent's
    // base frame, BUT DO NOT change their inertias (they remain
    // expressed in the moving link's frame).
    //
    // XXXX should take care to unhook them from their parents, which
    // should really get deleted after this...
    for (forest_t::iterator subtree(fixed_subtrees.begin()); subtree != fixed_subtrees.end(); ++subtree) {
      deFrame const & old_parent_frame((*subtree)->tao_home_frame);
      for (forest_t::iterator grandchild((*subtree)->children.begin());
	   grandchild != (*subtree)->children.end();
	   ++grandchild)
	{
	  deFrame relocated_home_frame;
	  relocated_home_frame.multiply(old_parent_frame, (*grandchild)->tao_home_frame);
	  (*grandchild)->tao_home_frame = relocated_home_frame;
	  (*grandchild)->parent = fused_duplicate;
	  fused_duplicate->children.insert(*grandchild);
	}
    }
    
    return fused_duplicate;
  }
  
  
  /** \note In the fused tree, inertias are expressed wrt COM, but TAO
      wants them wrt link origin (after the joint). */
  void create_tao_tree(taoDNode * tao_parent,
		       element const * child,
		       std::vector<std::string> & id_to_link_name,
		       std::vector<std::string> * id_to_joint_name,
		       std::vector<double> * joint_limit_lower,
		       std::vector<double> * joint_limit_upper) throw(std::runtime_error)
  {
    std::string const & name(child->urdf_link->name);
    urdf::Joint const * urdf_joint(child->urdf_link->parent_joint.get());
    taoJoint * tao_joint(0);
    if ( ! urdf_joint) {
      throw std::runtime_error("create_tao_tree(): urdf_link `" + name + "' has no joint");
    }
    else {
      
      switch (urdf_joint->type) {

      case urdf::Joint::UNKNOWN:
	throw std::runtime_error("create_tao_tree(): joint of urdf_link `" + name + "' is of UNKNOWN type");
	break;

      case urdf::Joint::REVOLUTE:
      case urdf::Joint::PRISMATIC:
      case urdf::Joint::CONTINUOUS: // this is just a revolute joint without limits
	{
	  // Grrrrr, TAO can only handle X, Y, or Z axes, not
	  // arbitraty ones. So let's see if the URDF looks like it
	  // has one of those, otherwise bail out. We'd have to extend
	  // TAO in order to handle the general case (easily done, but
	  // it's just annoying that TAO cannot do it out of the box).
	  taoAxis foo;
	  urdf::Vector3 const & bar(urdf_joint->axis);
	  if ((fabs(bar.x - 1) < 1e-3) && (fabs(bar.y) < 1e-3) && (fabs(bar.z) < 1e-3))
	    foo = TAO_AXIS_X;
	  else if ((fabs(bar.x) < 1e-3) && (fabs(bar.y - 1) < 1e-3) && (fabs(bar.z) < 1e-3))
	    foo = TAO_AXIS_Y;
	  else if ((fabs(bar.x) < 1e-3) && (fabs(bar.y) < 1e-3) && (fabs(bar.z - 1) < 1e-3))
	    foo = TAO_AXIS_Z;
#define ALLOW_NEGATIVE_AXES
#ifdef ALLOW_NEGATIVE_AXES
	  else if ((fabs(bar.x + 1) < 1e-3) && (fabs(bar.y) < 1e-3) && (fabs(bar.z) < 1e-3))
	    foo = TAO_AXIS_X;
	  else if ((fabs(bar.x) < 1e-3) && (fabs(bar.y + 1) < 1e-3) && (fabs(bar.z) < 1e-3))
	    foo = TAO_AXIS_Y;
	  else if ((fabs(bar.x) < 1e-3) && (fabs(bar.y) < 1e-3) && (fabs(bar.z + 1) < 1e-3))
	    foo = TAO_AXIS_Z;
#endif // ALLOW_NEGATIVE_AXES
	  else {
	    std::ostringstream os;
	    os << "create_tao_tree(): sorry but TAO cannot handle the joint axis (" << bar.x << "  " << bar.y
	       << "  " << bar.z << ") of urdf_link `" << name << "'";
	    throw std::runtime_error(os.str());
	  }
	  
	  if (urdf::Joint::REVOLUTE == urdf_joint->type) {
	    LOG_DEBUG (logger, "create_tao_tree(): REVOLUTE joint");
	    tao_joint = new taoJointRevolute(foo);
	  }
	  else if (urdf::Joint::CONTINUOUS == urdf_joint->type) {
	    LOG_DEBUG (logger, "create_tao_tree(): CONTINUOUS joint (treated like a revolute joint<)");
	    tao_joint = new taoJointRevolute(foo);
	  }
	  else if (urdf::Joint::PRISMATIC == urdf_joint->type) {
	    LOG_DEBUG (logger, "create_tao_tree(): PRISMATIC joint");
	    tao_joint = new taoJointPrismatic(foo);
	  }
	  
	  if ( ! tao_joint) {
	    throw std::runtime_error("BUG in create_tao_tree(): no TAO joint after URDF->TAO joint type switch");
	  }
	  
	  taoVarDOF1 * dof(new taoVarDOF1());
	  dof->_Q = 0;
	  dof->_dQ = 0;
	  dof->_ddQ = 0;
	  dof->_Tau = 0;
	  tao_joint->setDVar(new taoVarDOF1());
	}
	break;

      case urdf::Joint::FLOATING:
	throw std::runtime_error("create_tao_tree(): unsupported FLOATING joint type for urdf_link `"
				 + name + "'");
	break;

      case urdf::Joint::PLANAR:
	throw std::runtime_error("create_tao_tree(): unsupported PLANAR joint type for urdf_link `"
				 + name + "'");
	break;

      case urdf::Joint::FIXED:
	throw std::runtime_error("create_tao_tree(): unsupported FIXED joint type for urdf_link `"
				 + name + "'");
	break;

      default:
	{
	  std::ostringstream os;
	  os << "create_tao_tree(): invalid joint type #" << urdf_joint->type
	     << " for urdf_link `" << name << "'";
	  throw std::runtime_error(os.str());
	}

      }
    }
    
    if ( ! tao_joint) {
      throw std::runtime_error("BUG in create_tao_tree(): no TAO joint after switch, should have bailed earlier");
    }
    
    tao_joint->reset();
    tao_joint->setDamping(0.0); // could be taken from URDF
    tao_joint->setInertia(0.0); // could be taken from URDF
    
    // NOTE: this form of the taoNode ctor takes care of linking the
    // new instance into the child/sibling structure, so there is no
    // need to add the new node to the parent.  (Which incidentally
    // also is the reason why this function need not return the
    // freshly created instance).
    taoNode * tao_node(new taoNode(tao_parent, &(child->tao_home_frame)));
    tao_node->setID(id_to_link_name.size());
    id_to_link_name.push_back(name);
    tao_node->addJoint(tao_joint); 
    tao_node->addABNode();
    if (id_to_joint_name) {
      id_to_joint_name->push_back(urdf_joint->name);
    }
    if (joint_limit_lower) {
      if (( ! urdf_joint->limits) || (urdf::Joint::CONTINUOUS == urdf_joint->type)) {
	joint_limit_lower->push_back(std::numeric_limits<double>::min());
      }
      else {
	joint_limit_lower->push_back(urdf_joint->limits->lower);
      }
    }
    if (joint_limit_upper) {
      if (( ! urdf_joint->limits) || (urdf::Joint::CONTINUOUS == urdf_joint->type)) {
	joint_limit_upper->push_back(std::numeric_limits<double>::max());
      }
      else {
	joint_limit_upper->push_back(urdf_joint->limits->upper);
      }
    }
    
    // Yes, this looks weird, doesn't it? Don't worry, it's just TAO's
    // way of copying mass properties.
    child->tao_mass_prop.get(tao_node->mass(), tao_node->center(), tao_node->inertia());
    
    // And then we have to express the inertia wrt to the link origin
    inertia_parallel_axis_transform(*tao_node->inertia(),
				    *child->tao_mass_prop.center(),
				    *child->tao_mass_prop.mass(),
				    *tao_node->inertia());
    
    for (forest_t::const_iterator grandchild(child->children.begin());
	 grandchild != child->children.end();
	 ++grandchild)
      {
	create_tao_tree(tao_node, *grandchild, id_to_link_name, id_to_joint_name, joint_limit_lower, joint_limit_upper);
      }
  }
  
  
  element * find_element_by_name(element * root, std::string const & name)
  {
    if (root->getName() == name)
      return root;
    element * other(0);
    for (forest_t::const_iterator child(root->children.begin()); child != root->children.end(); ++child) {
      other = find_element_by_name(*child, name);
      if (other)
	break;
    }
    return other;
  }
  
  
  void compute_global_frame(element const * child, deFrame & global_frame)
  {
    if ( ! child) {
      global_frame.identity();
      return;
    }
    global_frame = child->tao_home_frame;
    for (child = child->parent; child; child = child->parent) {
      deFrame tmp;
      tmp.multiply(child->tao_home_frame, global_frame);
      global_frame = tmp;
    }
  }
  
  
  taoNodeRoot * convert(urdf::Model const & urdf_model,
			std::string const & tao_root_name,
			LinkFilter const & link_filter,
			std::vector<std::string> * tao_id_to_link_name_map,
			std::vector<std::string> * tao_id_to_joint_name_map,
			std::vector<double> * joint_limit_lower,
			std::vector<double> * joint_limit_upper) throw(std::runtime_error)
  {
    Converter converter(link_filter);
    
    element * unfused_root(converter.read_urdf_tree(0, urdf_model.getRoot()));
    element * fused_root(converter.copy_fuse_fixed(0, unfused_root));
    
    element * conversion_root(find_element_by_name(fused_root, tao_root_name));
    if ( ! conversion_root) {
      ostringstream msg;
      msg << "urdf_to_tao::convert(): no link called `" << tao_root_name << "' in the fused URDF model\n"
	  << "  Note that custom link filters might remove links during the fusion process\n"
	  << "  Here's the fused tree:\n";
      dump_tree(msg, fused_root, "  ", false);
      throw runtime_error(msg.str());
    }
    
    deFrame global_frame;
    compute_global_frame(conversion_root, global_frame);
    taoNodeRoot * tao_root(new taoNodeRoot(&global_frame));
    tao_root->setID(-1);
    
    std::vector<std::string> id_to_link_name; // we need this anyway for assigning IDs
    if (0 == tao_id_to_link_name_map)
      tao_id_to_link_name_map = &id_to_link_name;
    
    // We do not want the conversion_root itself to be descendend of the
    // TAO root, we want all its children to descend from it. Otherwise,
    // the first joint that moves is the one which attaches the
    // conversion_root to its parent, which is probably not what you
    // want... you want the conversion_root to be fixed in space.
    for (forest_t::const_iterator child(conversion_root->children.begin());
	 child != conversion_root->children.end();
	 ++child)
      {
	create_tao_tree(tao_root, *child, *tao_id_to_link_name_map,
			tao_id_to_joint_name_map, joint_limit_lower, joint_limit_upper);
      }
    
    return tao_root;
  }
  
}
