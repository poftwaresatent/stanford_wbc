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
 \file       CSkeletonLinkNew.h (provides a template for a skeletal link)
 */
//=========================================================================


#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <fstream>

#include "CWrapCylinderNew.h"
#include "CWrapEllipsoidNew.h"
#include "CJointNew.h"

namespace osimparser {

using namespace std;
class CSkeletonLinkNew {
public:
  string name;
  double mass, inertia_xx, inertia_yy, inertia_zz, inertia_xy, inertia_xz,
      inertia_yz;
  PrVector3 mass_center;
  PrMatrix3 inertia;
  PrVector3 scale_factors;
  vector<string> vtpGeometryFiles;

  int parent_id; //Index of parent in vSimmBodyVector
  int self_id; //Index of self in vSimmBodyVector

  vector<CWrapCylinderNew> wrapCylinderVector;
  vector<CWrapEllipsoidNew> wrapEllipsoidVector;
  vector<CJointNew> jointVector;

  double display_preference;
  bool show_normals, show_axes;
  string material_name;
  //TiXmlElement pBodySetObjectElement;

public:
  CSkeletonLinkNew(void);
  ~CSkeletonLinkNew(void);

  bool loadOsimBodySetObject(TiXmlElement* element);
  bool loadOsimWrapObjectSetObjects(TiXmlElement* pWrapObjectSetObjects);
};

}
