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
 \file       SkeletonCoordinate.h (contains skeletal coordinate information)
 */
//=========================================================================

#pragma once

#include <string>
#include <iostream>
#include <vector>

#include "../OsimHeaders.hpp"

namespace osimparser {
using namespace std;
using namespace wbc_tinyxml;

class CSkeletonCoordinate {
  string name, keys;
  double range[2], default_value, value, tolerance, stiffness;
  bool clamped, locked, restraint_active;

public:
  CSkeletonCoordinate(void);
  ~CSkeletonCoordinate(void);
  bool loadOsimCoordSetObject(TiXmlElement* pSimmCoordinateElement);
};

}
