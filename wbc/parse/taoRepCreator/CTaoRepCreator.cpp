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
 \file       CTaoRepCreator.cpp
 */
//=========================================================================

#include "CTaoRepCreator.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));

using namespace std;

namespace wbc_representations {

CTaoRepCreator::CTaoRepCreator() {
}

CTaoRepCreator::~CTaoRepCreator() {
}

/**Creates a tao branching representation out of a robot definition
 * supplied by the robotarchitect.
 * Arguments:
 * robotarchitect::CRobotDefinition<SControllerRobotLink>* -- The robot defintion
 * deFrame -- The home frame
 * int -- Whether the root node is fixed or not
 * int -- The robot id of the root node in the robot definition root-vector.*/
taoNodeRoot* CTaoRepCreator::taoRootRepCreator(
		robotarchitect::CRobotDefinition<SControllerRobotLink>* arg_robdef_p,
		const int arg_robot_id) {
	taoNodeRoot* tmp_taoRoot_p; //Returns a root node
	SControllerRobotLink* tmp_robdefRoot_p;

	//********Step 1***********
	//Check if a valid representation was passed.
	//*************************
	if (arg_robdef_p == NULL) {
		//robot_logger<<"\n taoRootRepCreator: NULL robot definition supplied to taoRepCreator";
		return NULL;
	}
	if ((static_cast<int>((arg_robdef_p->retRootLinkVector())->size()) <= arg_robot_id) || (0
			> arg_robot_id)) {
		//robot_logger<<"\n taoRootRepCreator: No robot matches the id passed. Cannot build tao tree.";
		return NULL;
	}

	//********Step 2***********
	//Traverse the robotRoot's tree and construct a tao tree structure
	//*************************
	//Step 2a: Find the desired robot root.
	tmp_robdefRoot_p
			= (SControllerRobotLink*) (arg_robdef_p->retRootLinkVector())->at(
					arg_robot_id); //The root of the robot
	assert(NULL != tmp_robdefRoot_p);

	//Step 2b: Create the corresponding root tao node
	//Root node's home frame rotation and translation
	tmp_robdefRoot_p->homeFrame_.translation().zero();
	tmp_robdefRoot_p->homeFrame_.rotation().set(tmp_robdefRoot_p->rotAxis_, tmp_robdefRoot_p->rotAngle_);

	tmp_taoRoot_p = new taoNodeRoot(&tmp_robdefRoot_p->homeFrame_); //Define a new tao root node to return
	tmp_taoRoot_p->setIsFixed(tmp_robdefRoot_p->linkIsFixed_); // Is the node fixed or not
	tmp_taoRoot_p->setID((deInt) -1); //NOTE TODO What about the multiple robot case? Will all have rootId = -1

	//Assign the newly created tao root to the robot definition's root
	tmp_robdefRoot_p->taoNodeRootAddr = tmp_taoRoot_p;
	tmp_robdefRoot_p->taoNodeAddr = NULL;
	if (tmp_robdefRoot_p->parentAddr != NULL) {
	  LOG_WARN (logger, "@tapRepCreator: Root link has a parent -- setting to NULL.");
		tmp_robdefRoot_p->parentAddr = NULL;
	}

	//Step 2c:
	createChildTaoNodes(tmp_robdefRoot_p, tmp_robdefRoot_p->taoNodeRootAddr);

	//********Step 3***********
	//Initialize TaoDynamics and return the root of the tao tree
	//*************************
	taoDynamics::initialize(tmp_taoRoot_p);

	tmp_robdefRoot_p = NULL;//Just to be safe we set these back to NULL

	return tmp_taoRoot_p; //Return the created tao root structure
}

/**This function creates child tao nodes for a given link in the robot's
 * definition. */
void CTaoRepCreator::createChildTaoNodes(
		SControllerRobotLink* arg_parentLink_p, taoNodeRoot* arg_taoRoot) {
	vector<SControllerRobotLink*>::iterator childAddrVecIter, childAddrVecIterE;
	//Traverse the list of child links and create a taoNode for each one
	for (childAddrVecIter = arg_parentLink_p->childAddrVector.begin(), childAddrVecIterE
			= arg_parentLink_p->childAddrVector.end(); childAddrVecIter
			!= childAddrVecIterE; ++childAddrVecIter) {
		(*childAddrVecIter)->taoNodeRootAddr = arg_taoRoot;
		createTaoNonRootNode((*childAddrVecIter));
		createChildTaoNodes((*childAddrVecIter), arg_taoRoot);
	}
}

/**This function initializes a node to be added to the tree being created
 * private: returns a taoNode*
 *
 * Sets the following parameters (stored in arg_linkAddrTao) for the
 * new taoNode:
 * 1. Link to its parent
 * 2. Its frame of reference wrt its parent
 * 3. Its physical link properties (mass, inertia etc)
 * 4. The node's joint(s) -- typically each node represents <1 link,1 joint>
 */
void CTaoRepCreator::createTaoNonRootNode(
		SControllerRobotLink* arg_thisLinkAddr) {
	if (arg_thisLinkAddr->parentAddr == NULL) {
		//Root Node -- Not to be initialized here.
		arg_thisLinkAddr->taoNodeAddr = NULL;
		return;
	} else {//Non-root Node
		taoNode* new_tao_node;
		if ((arg_thisLinkAddr->parentAddr->taoNodeAddr == NULL)
				&& (arg_thisLinkAddr->parentAddr->taoNodeRootAddr != NULL)) {//CASE: Parent is a root node.
			//1. Create a new tao node and link it to its parent
			new_tao_node = new taoNode(
					(taoDNode*) arg_thisLinkAddr->parentAddr->taoNodeRootAddr,
					&arg_thisLinkAddr->homeFrame_);
		} else {//CASE: Parent is a child node.
			//1. Create a new tao node and link it to its parent
			new_tao_node = new taoNode(
					(taoDNode*) arg_thisLinkAddr->parentAddr->taoNodeAddr,
					&arg_thisLinkAddr->homeFrame_);
		}
		//			else
		//			{
		//				//robot_logger<<"An error occoured while created a non-root child tao node. Its parent"
		//				//						<<" seems to be a root and a child node.";
		//				assert(false);
		//			}
		new_tao_node->setID(arg_thisLinkAddr->link_id);
		//2. Set up frame of reference
		deFrame comFrameLocal; //Set up local frame of reference
		comFrameLocal.identity();
		comFrameLocal.translation() = arg_thisLinkAddr->com_;

		//3. Set up the node's physical link properties
		deMassProp tmp_m; //Init mass, and set it in the new tao child node
		tmp_m.inertia(arg_thisLinkAddr->inertia_[0],
				arg_thisLinkAddr->inertia_[1], arg_thisLinkAddr->inertia_[2],
				&comFrameLocal);
		tmp_m.mass(arg_thisLinkAddr->mass_, &comFrameLocal);
		tmp_m.get(new_tao_node->mass(), new_tao_node->center(),
				new_tao_node->inertia());

		//  child_tao_node->setID((deInt) nodeID); //Not setting the node id : By design.

		// Create new joint for the new child node
		taoAxis tmp_axis; //The axis about which it rotates
		if (arg_thisLinkAddr->jointAxis_ == 0) {
			tmp_axis = TAO_AXIS_X;
		} else if (arg_thisLinkAddr->jointAxis_ == 1) {
			tmp_axis = TAO_AXIS_Y;
		} else {
			tmp_axis = TAO_AXIS_Z;
		}

		taoJointType tmp_type;
		if (arg_thisLinkAddr->jointType_ == 0) {
			tmp_type = TAO_JOINT_PRISMATIC;
		} else if (arg_thisLinkAddr->jointType_ == 1) {
			tmp_type = TAO_JOINT_REVOLUTE;
		} else if (arg_thisLinkAddr->jointType_ == 2) {
			tmp_type = TAO_JOINT_SPHERICAL;
		} else {
			tmp_type = TAO_JOINT_USER;
		}

		taoJoint* tmp_joint(0); //Create a new joint
		switch (tmp_type) {
		case TAO_JOINT_PRISMATIC:
			tmp_joint = new taoJointPrismatic(tmp_axis);
			tmp_joint->setDVar(new taoVarDOF1);
			break;
		case TAO_JOINT_REVOLUTE:
			tmp_joint = new taoJointRevolute(tmp_axis);
			tmp_joint->setDVar(new taoVarDOF1);
			break;
		case TAO_JOINT_SPHERICAL:
			tmp_joint = new taoJointSpherical();
			tmp_joint->setDVar(new taoVarSpherical);
			break;
		case TAO_JOINT_USER:
			break;
		}

		tmp_joint->reset(); //Reset the joint before it is added to the link
		tmp_joint->setDamping(0.0);
		tmp_joint->setInertia(0.0);

		new_tao_node->addJoint(tmp_joint);// Add joint to new_child_node

		new_tao_node->addABNode(); //Initialize geometry links

		arg_thisLinkAddr->taoNodeAddr = new_tao_node; //Set the new tao node's address
	}
}

}
