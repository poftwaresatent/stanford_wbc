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
   \file sai2lotus.cpp
   \author Roland Philippsen
*/

#include <jspace/test/sai_brep.hpp>
#include <jspace/test/sai_brep_parser.hpp>
#include <jspace/tao_dump.hpp>
#include <err.h>
#include <stdlib.h>

int main(int argc, char ** argv)
{
  if (argc < 2) {
    errx(EXIT_FAILURE, "SAI XML input file required");
  }
  try {
    jspace::test::BRParser brp;
    jspace::test::BranchingRepresentation * brep(brp.parse(argv[1]));
    jspace::tao_tree_info_s * tree(brep->createTreeInfo());
    dump_tao_tree_info_lotusxml(std::cout, "robot", "root", tree);
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "exception: %s", ee.what());
  }
}
