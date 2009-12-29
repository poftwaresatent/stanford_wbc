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

#include "CJointNew.h"

namespace osimparser {

CJointNew::CJointNew(void) {
  rotates_about_ = 2;
  translates_about_ = -1;
  default_pos_ = 0;
}

void CJointNew::init() {
}

CJointNew::~CJointNew(void) {
}

//===========================================================================
//  - PUBLIC METHOD -
/*!
 Load an osim SimmBody xml element into the class. This contains a
 description of a single joint.

 \fn       bool loadJoint( TiXmlElement* pSimmBodyElement )
 \param    pSimmBodyElement xml element containing a single Joint
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CJointNew::loadJoint(TiXmlElement* pCustomJoint) {//WrapCylinders and WrapEllipsoids
  TiXmlElement *levelSevenChildElement, *levelEightChildElement,
      *levelNineChildElement;
  bool loadFlag = false;
  const char *levelSevenChildTag, *levelEightChildTag, *levelNineChildTag;
  string sBuf;
  double temp0, temp1, temp2;//,*xyz_inertia;       2xs
  const char* str;

  // GET THE SEVENTH LEVEL CHILD
  levelSevenChildElement = pCustomJoint;
  levelSevenChildTag = levelSevenChildElement->Attribute("name");

  name = levelSevenChildTag;
  levelSevenChildTag = levelSevenChildElement->Value();
  if (strcmp(levelSevenChildTag, "CustomJoint") == 0) {
    isCustomJoint = true;
    isWeldJoint = false;
  }
  else {
    isCustomJoint = false;
    isWeldJoint = true;
  }

  levelEightChildElement = levelSevenChildElement->FirstChildElement(); //Actual links

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelEightChildElement != NULL) {//Loop iterates over level5 child elements <SimmBody>...
    levelEightChildTag = (levelEightChildElement->Value());
    if (strcmp(levelEightChildTag, "parent_body") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      parent_body = str;
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "location_in_parent") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      location_in_parent.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "orientation_in_parent") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      orientation_in_parent.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "location") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      location.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "orientation") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      orientation.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "sai_rot_axis") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      sscanf(str, "%d", &rotates_about_);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "sai_trans_axis") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      sscanf(str, "%d", &translates_about_);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "sai_default_pos") == 0) {
      str = levelEightChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &default_pos_);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "CoordinateSet") == 0 && isCustomJoint) {
      // GET THE SEVENTH LEVEL CHILD -- List of objects
      levelNineChildElement = levelEightChildElement->FirstChildElement(
          "objects");// objects
      levelNineChildTag = (levelNineChildElement->Value());
      if ((NULL == levelNineChildElement) || (strcmp(levelNineChildTag,
          "objects") != 0)) {
        loadFlag = false;
      }
      else {
        loadFlag = loadCoordinateObjects(levelNineChildElement);
      }
    }
    else if (strcmp(levelEightChildTag, "TransformAxisSet") == 0
        && isCustomJoint) {
      // GET THE SEVENTH LEVEL CHILD -- List of objects
      levelNineChildElement = levelEightChildElement->FirstChildElement(
          "objects");// objects
      levelNineChildTag = (levelNineChildElement->Value());
      if ((NULL == levelNineChildElement) || (strcmp(levelNineChildTag,
          "objects") != 0)) {
        loadFlag = false;
      }
      else {
        loadFlag = loadTransformAxisObjects(levelNineChildElement);
      }
    }
    else {
      loadFlag = true;
    }

    //                SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelEightChildElement = levelEightChildElement->NextSiblingElement();
  }
  // DONE
  convertEulerToAxisAngle();
  return true;
}

bool CJointNew::loadCoordinateObjects(TiXmlElement* coordinateObjects) {//Load in all Coordinate objects
  TiXmlElement *levelNineChildElement, *levelTenChildElement;
  bool loadFlag = false;
  const char *levelTenChildTag;

  CCoordObj *pCoordObj;

  levelNineChildElement = coordinateObjects;
  levelTenChildElement = levelNineChildElement->FirstChildElement();

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelTenChildElement != NULL) {//Loop iterates over level5 child elements <SimmBody>...
    levelTenChildTag = (levelTenChildElement->Value());
    if (strcmp(levelTenChildTag, "Coordinate") == 0) {
      pCoordObj = new CCoordObj();
      pCoordObj->loadCoordinateObject(levelTenChildElement);
      coordObjVector.push_back(*pCoordObj);
      loadFlag = true;
    }
    else {
      loadFlag = true;
    }
    //                SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelTenChildElement = levelTenChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

bool CJointNew::loadTransformAxisObjects(TiXmlElement* transformAxisObjects) {//Load in all transformAxis objects
  TiXmlElement *levelNineChildElement, *levelTenChildElement;
  bool loadFlag = false;
  const char *levelTenChildTag;

  CTransformAxisObj *pTransformAxisObj;

  levelNineChildElement = transformAxisObjects; //<objects>
  levelTenChildElement = levelNineChildElement->FirstChildElement(); //<TransformAxis name = "arb">

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelTenChildElement != NULL) {//Loop iterates over level5 child elements <SimmBody>...
    levelTenChildTag = (levelTenChildElement->Value());
    if (strcmp(levelTenChildTag, "TransformAxis") == 0) {
      pTransformAxisObj = new CTransformAxisObj();
      pTransformAxisObj->loadTransformAxisObject(levelTenChildElement);
      transformAxisVector.push_back(*pTransformAxisObj);
      loadFlag = true;
    }
    else {
      loadFlag = true;
    }
    //                SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelTenChildElement = levelTenChildElement->NextSiblingElement();
  }
  // DONE
  return true;

}

//Function: bool CJointNew::convertEulerToAxisAngle()
//Called at the end of object load: Converts the euler angle representation 
//into an axis-angle represenatation stored in:
//orientation_in_parent_axisangle.x, orientation_in_parent_axisangle.y, orientation_in_parent_axisangle.z, orientation_in_parent_axisangle.theta
//Assumes all angles are in radians.
//Returns: 
//true - rotation angle is not zero
//false - rotation angle is zero
bool CJointNew::convertEulerToAxisAngle() {
  Float eulerAngles[3];
  bool flag = true;
  double w, x, y, z, angle;//To go from euler angles -> quaternions -> axis-angle

  orientation_in_parent.getValues(eulerAngles, 3);

  double c1 = cos(eulerAngles[0] / 2);
  double s1 = sin(eulerAngles[0] / 2);
  double c2 = cos(eulerAngles[1] / 2);
  double s2 = sin(eulerAngles[1] / 2);
  double c3 = cos(eulerAngles[2] / 2);
  double s3 = sin(eulerAngles[2] / 2);

  w = c1 * c2 * c3 - s1 * s2 * s3;
  x = s1 * c2 * c3 + c1 * s2 * s3;
  y = c1 * s2 * c3 - s1 * c2 * s3;
  z = c1 * c2 * s3 + s1 * s2 * c3;

  angle = 2 * acos(w);
  double norm = x * x + y * y + z * z;
  if (norm < 0.001) { // when all euler angles are zero angle =0 so
    // we can set axis to anything to avoid divide by zero
    //Thus we have a zero degree rotation about axis [1 0 0]
    x = 1;
    y = z = 0;
    norm = 1;
    flag = false;
  }
  else {
    norm = sqrt(norm);
    flag = true;
  }

  orientation_in_parent_axisangle.x = x / norm;
  orientation_in_parent_axisangle.y = y / norm;
  orientation_in_parent_axisangle.z = z / norm;
  orientation_in_parent_axisangle.theta = angle;
  return flag;
}

}
