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

//=========================================================================
/*!
 \author     Samir Menon
 \file       CTaoRepCreator.hpp
 */
//=========================================================================

#ifndef CTAOREPCREATOR_HPP_
#define CTAOREPCREATOR_HPP_

#include <wbc/robarch/CRobotDefinition.hpp>
#include <wbc/robarch/glob_rob_ds/SControllerRobotLink.hpp>


namespace wbc_representations {

using namespace std;
using namespace robotarchitect;

/**This class translates robotarchitect::CRobotDefinition<> templates
 * into a vector of tao robots. It does not store any data.
 * This class will ideally be used once at the start of the program
 * to initialize the tao branching representation.*/
class CTaoRepCreator {
	/**Constructor */
	CTaoRepCreator();
public:
	~CTaoRepCreator();

	/**Creates a tao root structure out of a set of robot definitions
	 * input : CRobotDefinition<SControllerRobotLink>* 
	 * output : vector<taoNodeRoot*>
	 * Remember to dereference the vectors in taoNodeRoot* */
	static taoNodeRoot* taoRootRepCreator(robotarchitect::CRobotDefinition<
			SControllerRobotLink>* arg_robdef_p, const int arg_robot_id = 0);

private:
	/**Creates a tao node for a non-root link given an initialized home frame
	 * The node is added to the SControllerRobotLink* structure. */
	static void createTaoNonRootNode(SControllerRobotLink* arg_thisLinkAddr);
	static void createChildTaoNodes(SControllerRobotLink* arg_parentLink_p,
			taoNodeRoot* arg_taoRoot);
};

}

#endif //CTAOREPCREATOR_HPP_
