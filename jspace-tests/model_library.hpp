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

  }
}
