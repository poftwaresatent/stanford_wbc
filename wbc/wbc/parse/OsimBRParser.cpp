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
   \file OsimBRParser.cpp
   \author Samir Menon and Roland Philippsen
*/

#include "OsimBRParser.hpp"
#include <wbc/core/BranchingRepresentation.hpp>
#include <robarch/osimarchitect/COsimArchitect.hpp>
#include <robarch/rep_creators/CTaoRepCreator.hpp>
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {
  
  
  OsimBRParser::
  ~OsimBRParser()
  {
    delete robotDefinition_;
  }
  
  
  BranchingRepresentation * OsimBRParser::
  parse(const string& fileName)
    throw(std::runtime_error)
  {
    if (robotDefinition_)
      throw runtime_error("wbc::OsimBRParser::parse(): cannot parse multiple files");
    
    if (fileName == "")
      throw runtime_error("wbc::OsimBRParser::parse(): empty filename");
    
    robotarchitect::COsimArchitect tmp_RobArch;
    tmp_RobArch.readRobotDefinition(fileName);
    tmp_RobArch.buildRobotsFromLinks();
    robotDefinition_ = tmp_RobArch.returnControllerRobot();

    BranchingRepresentation * tmp_rep;
    tmp_rep = create(robotDefinition_);
    
    if (tmp_rep == NULL)
      throw runtime_error("wbc::OsimBRParser::parse(): Could not create tao representation from osim file "
			  + fileName);
    
    return tmp_rep;
  }
  
  
  BranchingRepresentation * OsimBRParser::
  create(robot_definition_t * arg_robdef_p)
  {
    BranchingRepresentation * ret_brRep_p(new BranchingRepresentation());
    taoNodeRoot* tmp_robotTaoRoot;
  
    // create tao root node (taoNodeRoot* rootNode_) from robdef* (robotDefinition_)
    //Presently building for robotid 0
    tmp_robotTaoRoot = wbc_representations::CTaoRepCreator::taoRootRepCreator(arg_robdef_p,0);
  
    // set ret_brRep_p's tao root node
    ret_brRep_p->rootNode_ = tmp_robotTaoRoot;
    
    // build the various maps and other sugar
//#warning 'XXXX this only works if the Osim file contains just one root, otherwise we need tree traversal starting at the root that represents our robot.'
    //typedef vector<robotarchitect::SControllerRobotLink*> vector<robotarchitect::SControllerRobotLink*>;
    vector<robotarchitect::SControllerRobotLink*> * linkvec(arg_robdef_p->getChildLinkVector());
    ret_brRep_p->numJoints_ = linkvec->size();
    ret_brRep_p->totalMass_ = 0;
    ret_brRep_p->grav_ = SAIVector(3);
    //ret_brRep_p->grav_ = SAIVector3((const Float*)arg_robdef_p->getGlobData()->gravity_);
    ret_brRep_p->grav_[0] = arg_robdef_p->getGlobData()->gravity_[0];
    ret_brRep_p->grav_[1] = arg_robdef_p->getGlobData()->gravity_[1];
    ret_brRep_p->grav_[2] = arg_robdef_p->getGlobData()->gravity_[2];

    ret_brRep_p->upperJointLimitVec_.setSize( ret_brRep_p->numJoints_ );
    ret_brRep_p->lowerJointLimitVec_.setSize( ret_brRep_p->numJoints_ );
  
    vector<int> actuation;
    int id(0);
    LOG_DEBUG (logger, "BranchingRepresentation::create(): creating branching sugar");
    for (vector<robotarchitect::SControllerRobotLink*>::const_iterator ii(linkvec->begin()), iend(linkvec->end());
	 ii != iend; ++ii, ++id) {
      taoDNode * node((*ii)->tao_node_addr_);
      
      LOG_DEBUG (logger,
		 "  " << id << ": " << node
		 << "  " << (*ii)->link_name_
		 << "  " << (*ii)->joint_name_
		 << ((*ii)->joint_is_free_ ? "  free-floating" : "  controlled"));
      
      ret_brRep_p->idToNodeMap_[id] = node;
      node->setID(id);
      
      LOG_DEBUG (logger, "  setting id:"<<id<<" for link:"<<(*ii)->link_name_);
      
      //Joint limits
      ret_brRep_p->upperJointLimitVec_[id] = (*ii)->joint_limit_upper_;
      ret_brRep_p->lowerJointLimitVec_[id] = (*ii)->joint_limit_lower_;
  	
      ret_brRep_p->linkNameToNodeMap_[(*ii)->link_name_] = node;
      ret_brRep_p->jointNameToNodeMap_[(*ii)->joint_name_] = node;
      
      ret_brRep_p->totalMass_ += (*ii)->mass_;
      if ( ! (*ii)->joint_is_free_)	// XXXX to do: maybe the other way around?
	actuation.push_back(id);
    }

    LOG_DEBUG (logger,
		   "Resizing unactuationMatrix to " << actuation.size()
		   << " by " << ret_brRep_p->numJoints_);
    
    //Build unactuation matrix: For selecting actuated joints
    ret_brRep_p->unactuationMatrix_.setSize(actuation.size(), ret_brRep_p->numJoints_, true);
    for (int ii(0); ii < static_cast<int>(actuation.size()); ++ii)
      ret_brRep_p->unactuationMatrix_.elementAt(ii, actuation[ii]) = 1.0;
    if (logger->isDebugEnabled()) {
      // the beauty of SAIMatrix... no way to tell it where to dump
      cout << "unactuationMatrix_:\n";
      ret_brRep_p->unactuationMatrix_.display();
    }
  
    return ret_brRep_p;
  }

}
