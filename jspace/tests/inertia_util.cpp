/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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
   \file inertia_util.cpp
   \author Roland Philippsen
*/

#include "inertia_util.hpp"
#include "tao_dump.hpp"
#include <tao/dynamics/tao.h>
#include <sstream>

using namespace std;

namespace jspace {
  
  
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
    //AAARGH no! copy-paste error! out_inertia += aa;
    out_inertia = aa;		// could directly to the ops in out_inertia, aa is just tmp
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
  
  
  void fuse_mass_properties(double orig_mass, deMatrix3 const & orig_inertia, deFrame const & orig_com,
			    double adtl_mass, deMatrix3 const & adtl_inertia, deFrame const & adtl_com,
			    deFrame const & home_of_adtl_wrt_orig,
			    double & fused_mass, deMatrix3 & fused_inertia, deFrame & fused_com)
  {
    //// LOG_INFO (logger,
    //// "      fuse_mass_properties():\n"
    //// << "        orig: m = " << orig_mass << "  I = " << inertia_matrix_to_string(orig_inertia)
    //// << "  COM = " << orig_com << "\n"
    //// << "        adtl: m = " << adtl_mass << "  I = " << inertia_matrix_to_string(adtl_inertia)
    //// << "  COM = " << adtl_com << "\n"
    //// << "        home_of_adtl_wrt_orig = " << home_of_adtl_wrt_orig);
    
    //////////////////////////////////////////////////
    // 1. compute new total mass
    
    //// LOG_DEBUG (logger, "        1. compute new total mass");
    fused_mass = orig_mass + adtl_mass;
    //// LOG_DEBUG (logger,
    //// "           fused_mass = orig_mass + adtl_mass = " << orig_mass << " + " << adtl_mass
    //// << " = " << fused_mass);
    
    //////////////////////////////////////////////////
    // 2. compute new COM
    
    //// LOG_DEBUG (logger, "        2. compute new COM");
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
    //// LOG_DEBUG (logger,
    //// "           adtl_com_wrt_o = " << adtl_com_wrt_o << "\n"
    //// << "           fused_com = " << fused_com.translation());
    
    //////////////////////////////////////////////////
    // 3. parallel axis transform of original inertia to new COM
    
    //// LOG_DEBUG (logger, "        3. parallel axis transform of original inertia to new COM");
    deVector3 orig_to_new_com;
    orig_to_new_com = fused_com.translation();
    orig_to_new_com -= orig_com.translation();
    deMatrix3 orig_inertia_at_new_com; // can probably reuse same instance
    inertia_parallel_axis_transform(orig_inertia, orig_to_new_com, orig_mass, orig_inertia_at_new_com);
    //// LOG_DEBUG (logger,
    //// "           orig_to_new_com = " << orig_to_new_com << "\n"
    //// << "           orig_inertia_at_new_com = " << inertia_matrix_to_string(orig_inertia_at_new_com));
    
    //////////////////////////////////////////////////
    // 4. similarity transform of additional inertia to original axes
    
    //// LOG_DEBUG (logger, "        4. similarity transform of additional inertia to original axes");
    deMatrix3 adtl_inertia_at_new_com;
    inertia_similarity_transform(adtl_inertia,
				 home_of_adtl_wrt_orig.rotation(), // XXXX ignore possible adtl com frame rot
				 adtl_inertia_at_new_com);
    //// LOG_DEBUG (logger,
    //// "           home_of_adtl_wrt_orig = " << home_of_adtl_wrt_orig << "\n"
    //// << "           (intermediate) adtl_inertia_at_new_com = "
    //// << inertia_matrix_to_string(adtl_inertia_at_new_com));
    
    //////////////////////////////////////////////////
    // 5. parallel axis of additional inertia to new COM (delta COM expressed in original frame)
    //    ---> translate by the delta of old to new COM_adtl in orig frame
    
    //// LOG_DEBUG (logger, "        5. parallel axis of additional inertia to new COM");
    deVector3 adtl_to_new_com;
    adtl_to_new_com = adtl_com_wrt_o.translation();
    adtl_to_new_com -= fused_com.translation();
    inertia_parallel_axis_transform(adtl_inertia, adtl_to_new_com, adtl_mass, adtl_inertia_at_new_com);
    //// LOG_DEBUG (logger,
    //// "           adtl_to_new_com = " << adtl_to_new_com << "\n"
    //// << "           (final) adtl_inertia_at_new_com = "
    //// << inertia_matrix_to_string(adtl_inertia_at_new_com));
    
    //////////////////////////////////////////////////
    // 6. add inertias
    //    ---> add inertia due to the point masses at the COM?
    //    ---> no, masses already taken into account during parallel axis theorem application!
    
    //// LOG_DEBUG (logger, "        6. add inertias");
    fused_inertia = orig_inertia_at_new_com;
    fused_inertia += adtl_inertia_at_new_com;
    //// LOG_INFO (logger, "           fused_inertia = " << inertia_matrix_to_string(fused_inertia));
  }
  
  
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
  
  
  void mass_inertia_explicit_form(Model const & model, Matrix & mass_inertia,
				  std::ostream * dbgos)
    throw(std::runtime_error)
  {
    size_t const ndof(model.getNDOF());
    mass_inertia = Matrix::Zero(ndof, ndof);
    
    if (dbgos) {
      *dbgos << "jspace::mass_inertia_explicit_form()\n"
	     << "  ndof = " << ndof << "\n";
    }
    
    for (size_t ii(0); ii < ndof; ++ii) {
      
      if (dbgos) {
	*dbgos << "  computing contributions of node " << ii << "\n";
      }
      
      taoDNode * node(model.getNode(ii));
      if ( ! node) {
	ostringstream msg;
	msg << "jspace::mass_inertia_explicit_form(): no node for index " << ii;
	throw runtime_error(msg.str());
      }
      
      Transform global_com;
      Matrix Jacobian;
      deVector3 const * com(node->center());
      if (com) {
	if ( ! model.computeGlobalFrame(node, com->elementAt(0), com->elementAt(1), com->elementAt(2), global_com)) {
	  ostringstream msg;
	  msg << "jspace::mass_inertia_explicit_form(): computeGlobalFrame() of COM failed for index " << ii;
	  throw runtime_error(msg.str());
	}
      }
      else {
	// pretend the COM is at the node origin
	if ( ! model.getGlobalFrame(node, global_com)) {
	  ostringstream msg;
	  msg << "jspace::mass_inertia_explicit_form(): getGlobalFrame() failed for index " << ii;
	  throw runtime_error(msg.str());
	}
      }
      if ( ! model.computeJacobian(node, global_com.translation(), Jacobian)) {
	ostringstream msg;
	msg << "jspace::mass_inertia_explicit_form(): computeJacobian() of COM failed for index " << ii;
	throw runtime_error(msg.str());
      }
      
      if (dbgos) {
	if (com) {
	  *dbgos << "    local COM: " << *com << "\n";
	}
	else {
	  *dbgos << "    local COM: null\n";
	}
	*dbgos << "    global COM: " << pretty_string(global_com.translation()) << "\n"
	       << "    Jacobian:\n" << pretty_string(Jacobian, "        ");
      }
      
      if (Jacobian.rows() != 6) {
	ostringstream msg;
	msg << "jspace::mass_inertia_explicit_form(): Jacobian of node " << ii
	    << " has " << Jacobian.rows() << " rows instead of 6";
	throw runtime_error(msg.str());
      }
      if (static_cast<size_t>(Jacobian.cols()) != ndof) {
	ostringstream msg;
	msg << "jspace::mass_inertia_explicit_form(): Jacobian of node " << ii
	    << " has " << Jacobian.cols() << " columns instead of " << ndof;
	throw runtime_error(msg.str());
      }
      
      // A problem that we will be having with TAO's inertia info a
      // bit further down is that it contains the contribution from
      // the mass, and we have to remove that before proceeding with
      // the rotational contribution to the mass-inertia matrix. Thus
      // the introduction of Im.
      
      Eigen::Matrix3d Im(Eigen::Matrix3d::Zero());
      deFloat const * mass(node->mass());
      if (mass) {
	Matrix const mass_contrib(Jacobian.block(0, 0, 3, ndof).transpose() * Jacobian.block(0, 0, 3, ndof));
	mass_inertia += *mass * mass_contrib;
	if (com) {
	  double const xx(pow(com->elementAt(0), 2));
	  double const yy(pow(com->elementAt(1), 2));
	  double const zz(pow(com->elementAt(2), 2));
	  double const xy(com->elementAt(0) * com->elementAt(1));
	  double const xz(com->elementAt(0) * com->elementAt(2));
	  double const yz(com->elementAt(1) * com->elementAt(2));
	  Im <<
	    yy+zz, -xy, -xz,
	    -xy, zz+xx, -yz,
	    -xz, -yz, xx+yy;
	  Im *= *mass;
	}
	if (dbgos) {
	  *dbgos << "    mass: " << *mass << "\n"
		 << "    JvT x Jv:\n" << pretty_string(mass_contrib, "        ")
		 << "    contribution of mass:\n" << pretty_string(*mass * mass_contrib, "        ")
		 << "    Im (for subsequent use):\n" << pretty_string(Im, "        ");
	}
      }
      else if (dbgos) {
	*dbgos << "    no mass\n";
      }
      
      // I think this is expressed wrt node origin, not COM. But in
      // any case, what matters is the instantaneous rotational
      // velocity due to joint ii, which is the same throughout the
      // entire link. Given that the COM frame is assumed to be
      // aligned with the node origin frame anyway, all we have to do
      // is express the rotational contribution of the Jacobian in the
      // local frame and use that.
      
      deMatrix3 const * inertia(node->inertia());
      if (inertia) {
	
	// NOTE: global_com.rotation() would require SVD, but we know
	// that we are dealing with an affine transform, so taking
	// what Eign2 calls the "linear" part is fine, because that's
	// simply the 3x3 upper left block of the homogeneous
	// transformation matrix. Hopefully anyway.
	
	// Use the transpose to get the inverse of the rotation.
	Matrix const J_omega(global_com.linear().transpose() * Jacobian.block(3, 0, 3, ndof));
	
	Eigen::Matrix3d Ic;
	Ic <<
	  inertia->elementAt(0, 0), inertia->elementAt(0, 1), inertia->elementAt(0, 2),
	  inertia->elementAt(1, 0), inertia->elementAt(1, 1), inertia->elementAt(1, 2),
	  inertia->elementAt(2, 0), inertia->elementAt(2, 1), inertia->elementAt(2, 2);
	mass_inertia += J_omega.transpose() * (Ic - Im) * J_omega;
	if (dbgos) {
	  *dbgos << "    Ic:\n" << pretty_string(Ic, "        ")
		 << "    Ic - Im:\n" << pretty_string(Ic - Im, "        ")
		 << "    J_omega:\n" << pretty_string(J_omega, "        ")
		 << "    JoT x (Ic-Im) x Jo:\n"
		 << pretty_string(J_omega.transpose() * (Ic - Im) * J_omega, "        ");
	}
      }
      else if (dbgos) {
	*dbgos << "    no inertia\n";
      }
      
      if (dbgos) {
	*dbgos << "  mass_inertia so far:\n" << pretty_string(mass_inertia, "    ");
      }
    }
  }
  
}
