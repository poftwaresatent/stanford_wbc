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

/** \author Roland Philippsen */

#include "BRParser.hpp"
#include "TiXmlBRParser.hpp"
#include "OsimBRParser.hpp"


// just a quick hack for testing BRBuilder with a simple case
static wbc::BranchingRepresentation * create_builtin_puma() throw(std::runtime_error);


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
    try {
      if (0 == brp)
	throw runtime_error("wbc::BRParser::parse(): invalid parserName `" + parserName + "' (use `sai' or `osim')");
      BranchingRepresentation * br(brp->parse(fileName));
      return br;
    }
    catch (std::runtime_error const & ee) {
      if ("builtin:puma" == fileName) // quick hack!!!
	return create_builtin_puma();
      throw ee;
    }
  }
  
}


#include "BRBuilder.hpp"

using namespace wbc;

BranchingRepresentation * create_builtin_puma() throw(std::runtime_error)
{
  BRBuilder brb;
  brb.setRoot(0, 0, 0,
	      1, 0, 0, 0);
  int const base(brb.addNode("base",
			     0, 0, 0,	// com
			     34.40,	// mass
			     0, 0, 1.49)); // inertia
  int const upper_arm(brb.addNode("upper_arm",
				  0.068, 0.006, -0.016,
				  17.40,
				  0.13, 0.524, 5.249));
  int const lower_arm(brb.addNode("lower_arm",
				  0, -0.143, 0.014,
				  6.04,
				  0.192, 0.0154, 1.042));
  int const wrist_hand(brb.addNode("wrist_hand",
				   0, 0, -0.019,
				   0.82,
				   0.0018, 0.0018, 0.2013));
  int const wrist_finger(brb.addNode("wrist_finger",
				     0, 0, 0,
				     0.34,
				     0.0003, 0.0003, 0.1794));
  int const end_effector(brb.addNode("end_effector",
				     0, 0, 0.032,
				     0.09,
				     0.00015, 0.00015, 0.19304));
  brb.linkNode(base, -1,	// node, parent
	       0, 0, 0,		// translation
	       0, 0, 1, 0);	// rotation (axis + angle)
  brb.linkNode(upper_arm, base,
	       0, 0.2435, 0,
	       1, 0, 0, -1.57079632679489661923);
  brb.linkNode(lower_arm, upper_arm,
	       0.4318, 0, -0.0934,
	       1, 0, 0, 0);
  brb.linkNode(wrist_hand, lower_arm,
	       -0.0203, -0.4331, 0,
	       1, 0, 0, 1.57079632679489661923);
  brb.linkNode(wrist_finger, wrist_hand,
	       0, 0, 0,
	       1, 0, 0, -1.57079632679489661923);
  brb.linkNode(end_effector, wrist_finger,
	       0, 0, 0,
	       1, 0, 0, 1.57079632679489661923);
  brb.addJoint(base, "shoulder-yaw",   // node, name
	       0, -0.1, 0.1,  // default pos, lower limit, upper limit
	       'r', 'z');	// type, axis
  brb.addJoint(upper_arm, "shoulder-pitch",
	       0, -0.1, 0.1,
	       'r', 'z');
  brb.addJoint(lower_arm, "elbow",
	       0, -0.1, 0.1,
	       'r', 'z');
  brb.addJoint(wrist_hand, "wrist-roll1",
	       0, -0.1, 0.1,
	       'r', 'z');
  brb.addJoint(wrist_finger, "wrist-pitch",
	       0, -0.1, 0.1,
	       'r', 'z');
  brb.addJoint(end_effector, "wrist-roll2",
	       0, -0.1, 0.1,
	       'r', 'z');
  BranchingRepresentation * br(brb.create());
  return br;
}
