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
   \file sai_brep.hpp
   \author Luis Sentis (copy-paste-adapted by Roland Philippsen)
*/

#ifndef JSPACE_TESTS_SAI_BREP_HPP
#define JSPACE_TESTS_SAI_BREP_HPP

#include <jspace/tao_util.hpp>
#include <jspace/wrap_eigen.hpp>
#include <tao/matrix/TaoDeMath.h>	
#include <map>
#include <string>
#include <vector>

class taoDNode;
class taoNode;	
class taoNodeRoot;

namespace jspace {
  
  struct tao_tree_info_s;
  
  namespace test {
    
    class BranchingRepresentation {
      friend class BRParser;
      BranchingRepresentation();
      
    public:
      ~BranchingRepresentation();
      
      /** Conversion function for easier back-porting to newer code. You
	  should try to work with a jspace::tao_tree_info_s in the first
	  place... */
      jspace::tao_tree_info_s * createTreeInfo();
    
    private:
      taoNodeRoot* rootNode_; 
      idToNodeMap_t idToNodeMap_; 
      std::map<std::string, taoDNode*> linkNameToNodeMap_;
      std::map<std::string, taoDNode*> jointNameToNodeMap_;
      mutable std::map<std::string, taoDNode*> linkNameToNodeMapWithAliases_;
      mutable std::map<std::string, taoDNode*> jointNameToNodeMapWithAliases_;
      int numJoints_;
      double totalMass_;
      jspace::Vector grav_;
      jspace::Vector defaultJointPosVec_;
      jspace::Vector upperJointLimitVec_;
      jspace::Vector lowerJointLimitVec_;
      
      taoDNode* findNodeID( taoDNode*, int);
    };
    
  }
}

#endif // JSPACE_TESTS_SAI_BREP_HPP
