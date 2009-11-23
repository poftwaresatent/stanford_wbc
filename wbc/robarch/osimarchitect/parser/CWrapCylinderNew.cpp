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
 \file       CWrapCylinderNew.cpp (provides a template for a cylinder wrap)
 */
//=========================================================================

#include "CWrapCylinderNew.h"

namespace osimparser {

CWrapCylinderNew::CWrapCylinderNew(void) {
}

CWrapCylinderNew::~CWrapCylinderNew(void) {
}

//===========================================================================
//  - PUBLIC METHOD -
/*!
 Load an osim SimmBody xml element into the class. This contains a
 description of a single link (BodySet->objects->SimmBody)

 \fn       bool loadOsimBodySetObject( TiXmlElement* pSimmBodyElement )
 \param    pSimmBodyElement xml element containing a single SimmBody object
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CWrapCylinderNew::loadWrapCylinderObject(TiXmlElement* pSimmBodyElement) {
  //Load one osimBodySet object
  TiXmlElement *levelEightChildElement, *levelNineChildElement;
  bool loadFlag = true;
  const char *levelNineChildTag;
  string sVTPString, sBuf;
  double temp0, temp1, temp2;//,*xyz_inertia;
  const char* str;

  //        double xyz_inertia[9];

  // GET THE EIGTH LEVEL CHILD
  levelEightChildElement = pSimmBodyElement;
  levelNineChildElement = levelEightChildElement->FirstChildElement(); //Actual link data

  //Set name of link
  name = levelEightChildElement->Attribute("name"); //WrapCylinderName

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelNineChildElement != NULL) {//Loop iterates over level9 child elements <mass>...<mass_center>...<inertia>
    loadFlag = false;
    levelNineChildTag = (levelNineChildElement->Value());
    if (strcmp(levelNineChildTag, "xyz_body_rotation") == 0) {
      str = levelNineChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      xyz_body_rotation.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelNineChildTag, "translation") == 0) {
      str = levelNineChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      translation.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelNineChildTag, "quadrant") == 0) {
      quadrant = levelNineChildElement->FirstChild()->Value();
      loadFlag = true;
    }
    else if (strcmp(levelNineChildTag, "active") == 0) {
      str = levelNineChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? active = true : active = false;
      loadFlag = true;
    }
    else if (strcmp(levelNineChildTag, "radius") == 0) {
      str = levelNineChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &radius);
      loadFlag = true;
    }
    else if (strcmp(levelNineChildTag, "length") == 0) {
      str = levelNineChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &length);
      loadFlag = true;
    }
    else if (strcmp(levelNineChildTag, "VisibleObject") == 0) {
      //NOTE: TODO: Implement this
      loadFlag = true;
    }
    else {
      //Replace with loadflag = false when exhaustive list of xml has been complied
      loadFlag = true;
    }

    //                assert( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelNineChildElement = levelNineChildElement->NextSiblingElement();
  }
  // DONE
  //delete xyz_inertia;
  return true;
}

}
