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
 \file       CLotusArchitect.cpp (opensim file parser)
 */
//=========================================================================
#ifdef TESTING_FUNCTIONS_ON
#include <iostream>
#endif 

#include "CLotusArchitect.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("robotarchitect"));

namespace robotarchitect {

bool CLotusArchitect::readRobotDefinition(const string arg_file, CRobotDefinition<SControllerRobotLink> & arg_cr_robdef)
{
  bool flag = true;
  buildCrRobot = true;

  return flag;
}



//Read in the graphics data from the xml file
bool CLotusArchitect::readRobotDefinition(const string arg_file, CRobotDefinition<SGraphicsRobotLink> & arg_gr_robdef)
{
  bool flag = true;
  buildGrRobot = true;

  return flag;
}

}
