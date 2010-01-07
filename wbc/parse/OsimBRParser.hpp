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
   \file OsimBRParser.hpp
   \author Samir Menon and Roland Philippsen
*/

#ifndef WBC_OSIM_PARSER_HPP
#define WBC_OSIM_PARSER_HPP

#include <wbc/parse/BRParser.hpp>
#include <wbc/robarch/CRobotDefinition.hpp>
#include <wbc/robarch/glob_rob_ds/SControllerRobotLink.hpp>

namespace wbc {

  /**
     \todo
     - Properly parse and initialize
       BranchingRepresentation::defaultJointPosVec_,
       BranchingRepresentation::upperJointLimitVec_, and
       BranchingRepresentation::lowerJointLimitVec_.
     - shouldn't we call taoDynamics::initialize() somewhere?
  */
  class OsimBRParser
    : public BRParser
  {
  public:
      OsimBRParser(){ robotDefinition_= NULL; }
    ~OsimBRParser();
    
    virtual BranchingRepresentation * parse(const std::string& fileName) throw(std::runtime_error);
    
  protected:
    typedef robotarchitect::CRobotDefinition<robotarchitect::SControllerRobotLink> robot_definition_t;
    static BranchingRepresentation * create(robot_definition_t * robdef);
    robot_definition_t * robotDefinition_;
  };
  
}

#endif // WBC_OSIM_PARSER_HPP
