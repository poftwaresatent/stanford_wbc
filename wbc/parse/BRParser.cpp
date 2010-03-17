/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file BRParser.cpp
   \author Roland Philippsen
*/

#include "BRParser.hpp"
#include "TiXmlBRParser.hpp"
#include "OsimBRParser.hpp"


namespace wbc {
  
  BranchingRepresentation * BRParser::
  parse(const std::string & parserName,
	const std::string & fileName)
    throw(std::runtime_error)
  {
    wbc::BRParser * brp(0);
    if (parserName.empty() || ("sai" == parserName))
      brp = new wbc::TiXmlBRParser();
    else if ("osim" == parserName)
      brp = new wbc::OsimBRParser();
    if (0 == brp)
      throw runtime_error("wbc::BRParser::parse(): invalid parserName `" + parserName + "' (use `sai' or `osim')");
    BranchingRepresentation * br(brp->parse(fileName));
    delete brp;
    return br;
  }
  
}
