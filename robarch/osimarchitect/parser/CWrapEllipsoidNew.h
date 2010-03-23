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
 \file       CWrapEllipsoidNew.h (provides a template for an ellipsoid wrap)
 */
//=========================================================================


#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <sstream>

#include "../OsimHeaders.hpp"

namespace osimparser {
using namespace std;
using namespace wbc_tinyxml;

class CWrapEllipsoidNew {
  string name, quadrant;
  SAIVector3 xyz_body_rotation, translation, dimensions;
  bool active;

public:
  CWrapEllipsoidNew(void);
  ~CWrapEllipsoidNew(void);

  bool loadWrapEllipsoidObject(TiXmlElement* element);
};

}
