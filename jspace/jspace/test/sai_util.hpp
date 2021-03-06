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
   \file jspace/test/sai_util.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_TEST_SAI_UTIL_HPP
#define JSPACE_TEST_SAI_UTIL_HPP

#include <stdexcept>
#include <string>

namespace jspace {
  class Model;
  namespace test {
    Model * parse_sai_xml_file(std::string const & filename,
			       bool enable_coriolis_centrifugal) throw(std::runtime_error);
  }
}

#endif // JSPACE_TEST_SAI_UTIL_HPP
