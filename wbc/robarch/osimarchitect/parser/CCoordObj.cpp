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
 \file       CCoordObj.cpp (provides a template for a skeletal coordinate)
 */
//=========================================================================

#include "CCoordObj.h"

namespace osimparser {

CCoordObj::CCoordObj(void) {
}

CCoordObj::~CCoordObj(void) {
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
bool CCoordObj::loadCoordinateObject(TiXmlElement* element) {//WrapCylinders and WrapEllipsoids
  TiXmlElement *levelTenChildElement, *levelElevenChildElement;
  bool loadFlag = false;
  const char *levelTenChildTag, *levelElevenChildTag;
  string sBuf;
  const char* str;

  // GET THE FIFTH LEVEL CHILD
  levelTenChildElement = element; //Gen Coordinate
  levelTenChildTag = levelTenChildElement->Value();

  name = levelTenChildTag;
  name = levelTenChildElement->Attribute("name");

  if (strcmp(levelTenChildTag, "Coordinate") != 0) {
    //            SAIAssertSz( false, "Error: Invalid XML format in loaded osim model at Coordinate element." );
    return false;
  }

  levelElevenChildElement = levelTenChildElement->FirstChildElement(); //Actual links

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelElevenChildElement != NULL) {//Loop iterates over level5 child elements <SimmBody>...
    levelElevenChildTag = (levelElevenChildElement->Value());
    if (strcmp(levelElevenChildTag, "default_value") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &default_value);
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "initial_value") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &initial_value);
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "tolerance") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &tolerance);
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "stiffness") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &stiffness);
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "range") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf", &rangeMin, &rangeMax);
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "keys") == 0) {
      //Implement this if needed
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "clamped") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? clamped = true : clamped = false;
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "locked") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? locked = true : locked = false;
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "restraint_function") == 0) {
      //NOTE: TODO: Figure out the use and implement it.
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "min_restraint_function") == 0) {
      //NOTE: TODO: Figure out the use and implement it.
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "max_restraint_function") == 0) {
      //NOTE: TODO: Figure out the use and implement it.
      loadFlag = true;
    }
    else {
      loadFlag = true;
    }

    //		SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelElevenChildElement = levelElevenChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

}
