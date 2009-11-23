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
 \file       SkeletonMarker.cpp (contains skeletal marker information)
 */
//=========================================================================

#include "SkeletonMarker.h"

namespace osimparser {

CSkeletonMarker::CSkeletonMarker(void) {
}

CSkeletonMarker::~CSkeletonMarker(void) {
}

//===========================================================================
//  - PRIVATE METHOD -
/*!
 Load an osim SimmKinematicsEngine xml element into the class. This contains a
 description of a marker (Marker)

 \fn       bool loadOsimMarkerSetObject( TiXmlElement* pSimmMarkerElement  )
 \param    pSimmMarkerElement xml element containing data to describe a marker
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CSkeletonMarker::loadOsimMarkerSetObject(TiXmlElement* pSimmMarkerElement) {
  //Load one osimMarkerSet object
  TiXmlElement *levelFiveChildElement, *levelSixChildElement,
      *levelSevenChildElement, *levelEightChildElement;
  bool loadFlag = true;
  const char *levelSixChildTag, *levelSevenChildTag;
  string sBuf;
  double temp0, temp1, temp2;
  const char* str;

  // GET THE FIFTH LEVEL CHILD
  levelFiveChildElement = pSimmMarkerElement;
  levelSixChildElement = levelFiveChildElement->FirstChildElement(); //Actual link data

  //Set name of marker
  name = levelFiveChildElement->Attribute("name"); //<Marker name="">

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelSixChildElement != NULL) {//Loop iterates over level6 child elements <mass>...<mass_center>...<inertia>
    loadFlag = false;
    levelSixChildTag = (levelSixChildElement->Value());
    if (strcmp(levelSixChildTag, "body") == 0) {/*<!--Body segment in the model on which the marker resides.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      body = str;
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "location") == 0) {/*<!--Location of a marker on the body segment.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      location.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "fixed") == 0) {
      /*<!--Flag (true or false) specifying whether or not a marker should be kept
       fixed in the marker placement step.  i.e. If false, the marker is
       allowed to move.-->*/
      str = levelSixChildElement->FirstChild()->Value();
      strcmp(str, "true") == 0 ? fixed = true : fixed = false;
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "VisibleObject") == 0) {
      // GET THE SIXTH LEVEL CHILD -> <geometry_files> f1.vtp f2.vtp f3.vtp...</>
      levelSevenChildElement = levelSixChildElement->FirstChildElement(); //vtp files
      while (levelSevenChildElement != NULL) {
        levelSevenChildTag = (levelSevenChildElement->Value());
        if (strcmp(levelSevenChildTag, "geometry_files") == 0) {
          //NOTE: TODO:
          //This never contains any geometry files so not handling it.
          //if(NULL!= levelSevenChildElement->FirstChild()->Value())
          //	str = levelSevenChildElement->FirstChild()->Value();
          ////Get the string containing the vtp files
          //stringstream ss(str);  // Insert the string into a stream
          //while (ss>>sBuf) //Reads streat till whitespace
          //{
          //	vtpGeometryFiles.push_back(sBuf);
          //}
        }
        else if (strcmp(levelSevenChildTag, "VisibleProperties") == 0) {
          levelEightChildElement = levelSevenChildElement->FirstChildElement(
              "display_preference");
          if (levelEightChildElement != NULL) {
            str = levelEightChildElement->FirstChild()->Value();
            sscanf(str, "%lf", &display_preference);
          }

          levelEightChildElement = levelSevenChildElement->FirstChildElement(
              "show_normals");
          if (levelEightChildElement != NULL) {
            str = levelEightChildElement->FirstChild()->Value();
            strcmp(str, "true") == 0 ? show_normals = true : show_normals
                = false;
          }

          levelEightChildElement = levelSevenChildElement->FirstChildElement(
              "show_axes");
          if (levelEightChildElement != NULL) {
            str = levelEightChildElement->FirstChild()->Value();
            strcmp(str, "true") == 0 ? show_axes = true : show_axes = false;
          }

          levelEightChildElement = levelSevenChildElement->FirstChildElement(
              "material_name");
          if (levelEightChildElement != NULL) {
            str = levelEightChildElement->FirstChild()->Value();
            material_name = str;
          }
        }
        else if (strcmp(levelSevenChildTag, "scale_factors") == 0) {
          str = levelSevenChildElement->FirstChild()->Value();
          sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
          scale_factors.values(temp0, temp1, temp2);
        }

        levelSevenChildElement = levelSevenChildElement->NextSiblingElement();
      }
      loadFlag = true;
    }
    else {
      //Replace with loadflag = false when exhaustive list of xml has been complied
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
