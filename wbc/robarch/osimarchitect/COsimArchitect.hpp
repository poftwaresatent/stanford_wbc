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
 \file       COsimArchitect.hpp (opensim file parser)
 */
//=========================================================================

#ifndef COSIMARCHITECT_HPP_
#define COSIMARCHITECT_HPP_

#include <wbc/robarch/CRobotArchitect.hpp>
#include <wbc/robarch/osimarchitect/parser/CSkeletonModelNew.h>

namespace robotarchitect {
using namespace osimparser;

class COsimArchitect: public robotarchitect::CRobotArchitect {
  osimparser::CSkeletonModelNew osimModel;
public:
  COsimArchitect();
  virtual ~COsimArchitect();
  /**Reads the osim file and adds its links to crRobotDef
   * and grRobotDef which are inherited from CRobotArchitect*/
  bool readRobotDefinition(const string arg_file);
};

}

#endif /*COSIMARCHITECT_HPP_*/
