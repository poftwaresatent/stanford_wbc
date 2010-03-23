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
 \file       CSkeletonModelNew.h (stores all data obtained from a skeletal model)
 */
//=========================================================================
#ifndef CSKELETONMODELNEW_H_
#define CSKELETONMODELNEW_H_

#include <iostream>
#include <string>

#include "CSkeletonLinkNew.h"
#include "SkeletonCoordinate.h"
#include "SkeletonMarker.h"
#include "CMuscleDefinition.h"

namespace osimparser {
using namespace std;

class CSkeletonModelNew {
public:
  //List of properties contained in the osim file.

  //a. Global properties
  string angle_units, length_units, force_units;
  SAIVector3 gravity; //0 -9.80665 0

  //osim xml file
  TiXmlDocument osimDocument;

  //b. Lists of objects to be rendered
  //list of objects
  vector<CSkeletonLinkNew> vSimmBodyVector;
  //list of coordinates
  vector<CSkeletonCoordinate> vSimmCoordinateVector;
  //list of joints
  //vector<CSkeletonJoint> vSimmJointVector; //Presently useless
  //list of markers
  vector<CSkeletonMarker> vSimmMarkerVector; //Presently useless
  //list of muscles
  vector<CMuscleDefinition> vMuscleDefVector;

public:
  CSkeletonModelNew(void);
  ~CSkeletonModelNew(void);

  bool loadOsimFile(const char * osimFileName);

private:
  bool loadSimbodyEngine(TiXmlElement *simmKinematicsEngineElement);
  bool loadBodySetObjects(TiXmlElement * bodySetObjectElement);
  bool loadConstraintSetObjects(TiXmlElement * constraintSetObjectElement);
  bool loadMarkerSetObjects(TiXmlElement * markerSetObjectElement);
  int findLinkId(string link_name);
  bool loadMuscleObjects(TiXmlElement * muscleElement);
};
}
#endif /* CSKELETONMODELNEW_H_ */
