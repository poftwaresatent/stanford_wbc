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
 \file       SkeletonCoordinate.cpp (contains skeletal coordinate information)
 */
//=========================================================================

#include "SkeletonCoordinate.h"

namespace osimparser {

CSkeletonCoordinate::CSkeletonCoordinate(void) {
}

CSkeletonCoordinate::~CSkeletonCoordinate(void) {
}

bool CSkeletonCoordinate::loadOsimCoordSetObject(
    TiXmlElement* pSimmCoordinateElement) {
  //Load one osimCoordinateSet object
  TiXmlElement *levelFiveChildElement, *levelSixChildElement;
  bool loadFlag = true;
  const char *levelSixChildTag;
  string sBuf;
  const char* str;

  // GET THE FIFTH LEVEL CHILD
  levelFiveChildElement = pSimmCoordinateElement;
  levelSixChildElement = levelFiveChildElement->FirstChildElement(); //Actual link data

  //Set name of link
  name = levelFiveChildElement->Attribute("name");

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelSixChildElement != NULL) {//Loop iterates over level6 child elements <mass>...<mass_center>...<inertia>
    loadFlag = false;
    levelSixChildTag = (levelSixChildElement->Value());
    if (strcmp(levelSixChildTag, "range") == 0) {
      /*<!--Allowed range for a coordinate.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf", &range[0], &range[1]);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "default_value") == 0) {
      /*<!--Default value for a coordinate.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &default_value);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "value") == 0) {
      /*<!--Value of the coordinate.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &value);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "tolerance") == 0) {
      /*<!--Tolerance for a coordinate.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &tolerance);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "stiffness") == 0) {
      /*<!--Stiffness of a coordinate.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &stiffness);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "keys") == 0) {
      /*<!--Computer keyboard keys that can be used to manipulate in a graphical
       application.-->
       Since this is presently useless I'm not including it.*/
      /*str = levelSixChildElement->FirstChild()->Value();
       keys =str;*/
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "clamped") == 0) {
      /*<!--Flag (true or false) indicating whether a coordinate is not permitted
       outside its range.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? clamped = true : clamped = false;
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "locked") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? locked = true : locked = false;
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "restraint_active") == 0) {
      /*<!--Flag (true or false) indicating whether or not the restraint function
       is active.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? restraint_active = true : restraint_active
          = false;
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "restraint_function") == 0) {
      /*<!--Pointer to a restraint function that applies generalized forces at a
       joint to keep the coordinate from moving outside its range.-->*/
      //NOTE TODO: restraint_function, min_restraint_function and max_restraint_function
      //are presently undefined in the code. Add handlers if they become useful.
      loadFlag = true;
    }
    else {
      //NOTE TODO: Change to false once an exhaustive list of xml tags has been built
      loadFlag = true;
    }

    //		assert( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelSixChildElement = levelSixChildElement->NextSiblingElement();
  }
  // DONE
  //delete xyz_inertia;
  return loadFlag;
}

}
