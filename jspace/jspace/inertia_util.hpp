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
   \file inertia_util.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_INERTIA_UTIL_HPP
#define JSPACE_INERTIA_UTIL_HPP

#include <jspace/Model.hpp>
#include <stdexcept>
#include <iosfwd>

class deMatrix3;
class deVector3;
class deQuaternion;
class deFrame;
class deMassProp;

namespace jspace {
  
  /**
     Parallel axis transform with fundamental types. The parallel axis
     transform allows you to find the inertia tensor with respect to a
     frame that has been translated (but not rotated) with respect to
     the frame that was used to express the inertia tensor.
  */
  void inertia_parallel_axis_transform(double in_ixx, double in_ixy, double in_ixz,
				       double in_iyy, double in_iyz, double in_izz,
				       double trans_x, double trans_y, double trans_z, double mass,
				       double & out_ixx, double & out_ixy, double & out_ixz,
				       double & out_iyy, double & out_iyz, double & out_izz);
  
  /**
     Parallel axis transform with TAO types.
  */
  void inertia_parallel_axis_transform(deMatrix3 const & in_inertia,
				       deVector3 const & translation, double mass,
				       deMatrix3 & out_inertia);
  
  /**
     Similarity transform with fundamental types. The similarity
     transform allows you to find the inertial tensor with respect to
     a frame that has been rotated (but not translated) with respect
     to the frame that was used to express the inertia tensor.
  */
  void inertia_similarity_transform(double in_ixx, double in_ixy, double in_ixz,
				    double in_iyy, double in_iyz, double in_izz,
				    double rot_qx, double rot_qy, double rot_qz, double rot_qw,
				    double & out_ixx, double & out_ixy, double & out_ixz,
				    double & out_iyy, double & out_iyz, double & out_izz);
  
  /**
     Similarity transform with a mixture of fundamental and TAO types.
  */
  void inertia_similarity_transform(double in_ixx, double in_ixy, double in_ixz,
				    double in_iyy, double in_iyz, double in_izz,
				    deQuaternion const & rotation,
				    deMatrix3 & out_inertia);
  
  
  /**
     Similarity transform with TAO types.
  */
  void inertia_similarity_transform(deMatrix3 const & in_inertia,
				    deQuaternion const & rotation,
				    /** OK to have &in_inertia == &out_inertia, we use copies. */
				    deMatrix3 & out_inertia);
  
  /**
     Fuse the mass properties (mass, center of mass, and tensor of
     inertia) of two bodies, one of them referred to as "original",
     the other as "additional". The result is computed with respect to
     the origin of the original body.  Inertia tensors are assumed to
     be expressed wrt to the center of mass. The axes of the COM
     frames are assumed to be aligned with the axes of their home
     frames (in other words, the rotational parts of orig_com and
     adtl_com are ignored).
  */
  void fuse_mass_properties(double orig_mass, deMatrix3 const & orig_inertia, deFrame const & orig_com,
			    double adtl_mass, deMatrix3 const & adtl_inertia, deFrame const & adtl_com,
			    deFrame const & home_of_adtl_wrt_orig,
			    double & fused_mass, deMatrix3 & fused_inertia, deFrame & fused_com);
  
  /**
     Fuse mass properties (see above for more details).
  */
  void fuse_mass_properties(deMassProp /*const*/ & original,
			    deMassProp /*const*/ & additional,
			    deFrame const & home_of_additional_wrt_original,
			    /** OK to pass same ref as original */
			    deMassProp & fused);
  
  void mass_inertia_explicit_form(Model const & model, Matrix & mass_inertia,
				  std::ostream * dbgos = 0)
    throw(std::runtime_error);
  
}

#endif // JSPACE_INERTIA_UTIL_HPP
