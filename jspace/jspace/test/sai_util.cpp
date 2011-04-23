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
   \file jspace/test/sai_util.cpp
   \author Roland Philippsen
*/

#include "sai_util.hpp"
#include "sai_brep_parser.hpp"
#include "sai_brep.hpp"
#include "../Model.hpp"

namespace jspace {
  namespace test {

    Model * parse_sai_xml_file(std::string const & filename,
			       bool enable_coriolis_centrifugal) throw(std::runtime_error)
    {
      test::BRParser brp;
      test::BranchingRepresentation * brep(brp.parse(filename));
      jspace::tao_tree_info_s * kg_tree(brep->createTreeInfo());
      jspace::tao_tree_info_s * cc_tree(0);
      if (enable_coriolis_centrifugal) {
	delete brep;
	brep = brp.parse(filename);
	cc_tree = brep->createTreeInfo();
      }
      delete brep;
    
      Model * model(new Model());
      std::ostringstream msg;
      if ( 0 != model->init(kg_tree, cc_tree, &msg)) {
	delete model;
	throw std::runtime_error("jspace::parse_sai_xml_file(" + filename
				 + "): model::init() failed: " + msg.str());
      }
      return model;
    }

  }  
}
