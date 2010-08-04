/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
#include <tao/dynamics/tao.h>

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

}
