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
 \file       CJointNew.cpp (provides a template for a skeletal joint)
 */
//=========================================================================


#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <math.h>

#include "../OsimHeaders.hpp"

#include "CCoordObj.h"
#include "CTransformAxisObj.h"

namespace osimparser {
using namespace std;
struct axis_angle {
  Float x, y, z, theta;
};

class CJointNew {
public:
  string name, quadrant, parent_body;
  bool isCustomJoint;
  bool isWeldJoint;
  PrVector3 location_in_parent, orientation_in_parent, location, orientation;
  bool active;
  int rotates_about_;
  int translates_about_;
  double default_pos_;

  axis_angle orientation_in_parent_axisangle;

  vector<CCoordObj> coordObjVector;
  vector<CTransformAxisObj> transformAxisVector;

public:
  CJointNew(void);
  ~CJointNew(void);

  void init();

  bool loadJoint(TiXmlElement* element);
  bool loadCoordinateObjects(TiXmlElement* coordinateObjects);
  bool loadTransformAxisObjects(TiXmlElement* transformAxisObjects);

  bool convertEulerToAxisAngle();
};

}
