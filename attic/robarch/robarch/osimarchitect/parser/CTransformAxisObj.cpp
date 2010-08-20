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
 \file       Transformaxis.cpp (provides a template for a axis transformations)
 */
//=========================================================================

#include "CTransformAxisObj.h"

namespace osimparser {

CTransformAxisObj::CTransformAxisObj(void) {
}

CTransformAxisObj::~CTransformAxisObj(void) {
}

//===========================================================================
//  - PUBLIC METHOD -
/*!
 Load an osim TransformAxisObject xml element into the class. This belongs to
 some joint.

 \fn       bool CTransformAxisObj::loadTransformAxisObject( TiXmlElement*
 transformAxisObject )
 \param    transformAxisObject xml element containing a single transform axis
 \return   Return true if object was loaded successfully
 */
//===========================================================================
bool CTransformAxisObj::loadTransformAxisObject(
    TiXmlElement* transformAxisObject) {//WrapCylinders and WrapEllipsoids
  TiXmlElement *levelTenChildElement, *levelElevenChildElement;
  bool loadFlag = false;
  const char *levelTenChildTag, *levelElevenChildTag;
  string sBuf;
  double temp0, temp1, temp2;//,*xyz_inertia;	2xs
  const char* str;

  // GET THE FIFTH LEVEL CHILD
  levelTenChildElement = transformAxisObject; //Transform axis for a joint
  levelTenChildTag = levelTenChildElement->Value();
  name = levelTenChildElement->Attribute("name");

  if (strcmp(levelTenChildTag, "TransformAxis") != 0) {
    //		assert( false, "Error: Invalid XML format in loaded osim model at TransFormAxis element." );
    return false;
  }

  levelElevenChildElement = levelTenChildElement->FirstChildElement(); //Actual links

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelElevenChildElement != NULL) {//Loop iterates over level11 child elements <SimmBody>...
    levelElevenChildTag = (levelElevenChildElement->Value());
    if (strcmp(levelElevenChildTag, "function") == 0) {
      //NOTE: TO BE DONE
      //Do something for this when needed
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "coordinate") == 0) {
      coordinate = levelElevenChildElement->FirstChild()->Value();
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "is_rotation") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? is_rotation = true : is_rotation = false;
      loadFlag = true;
    }
    else if (strcmp(levelElevenChildTag, "axis") == 0) {
      str = levelElevenChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      axis.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else {//For now ignore every other element
      loadFlag = true;
    }

    //		assert( loadFlag, "Error: Invalid osim model - Transform Axis." );

    // MOVE TO NEXT ELEMENT
    levelElevenChildElement = levelElevenChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

}
