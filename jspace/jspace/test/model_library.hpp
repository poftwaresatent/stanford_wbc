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
   \file model_library.hpp
   \author Roland Philippsen
*/

#include <jspace/Model.hpp>
#include <stdexcept>

namespace jspace {
  namespace test {
    
    class BranchingRepresentation;
    
    jspace::Model * create_puma_model() throw(std::runtime_error);
    jspace::Model * create_unit_mass_RR_model() throw(std::runtime_error);
    BranchingRepresentation * create_unit_mass_5R_brep() throw(std::runtime_error);
    jspace::Model * create_unit_mass_5R_model() throw(std::runtime_error);
    jspace::Model * create_unit_inertia_RR_model() throw(std::runtime_error);
    jspace::Model * create_unit_mass_RP_model() throw(std::runtime_error);
    
    jspace::Model * create_fork_4R_model() throw(std::runtime_error);
    
    /** q1...q4 are the joint angles in rad. o1...o4 are the node
	origins in global frame. c1...c4 are the COM positions in
	global frame. J1...J4 are the Jacobians at the node
	origins. */
    void compute_fork_4R_kinematics(double q1, double q2, double q3, double q4,
				    jspace::Vector & o1, jspace::Vector & o2, jspace::Vector & o3, jspace::Vector & o4,
				    jspace::Vector & com1, jspace::Vector & com2,
				    jspace::Vector & com3, jspace::Vector & com4,
				    jspace::Matrix & J1, jspace::Matrix & J2, jspace::Matrix & J3, jspace::Matrix & J4);
    
    void compute_unit_mass_RR_mass_inertia(double q1, double q2, jspace::Matrix & AA);
    void compute_unit_inertia_RR_mass_inertia(double q1, double q2, jspace::Matrix & AA);
    
  }
}
