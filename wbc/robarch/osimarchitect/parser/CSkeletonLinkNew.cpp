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
 \file       CSkeletonLinkNew.cpp (provides a template for a skeletal link)
 */
//=========================================================================

#include "CSkeletonLinkNew.h"

namespace osimparser {

CSkeletonLinkNew::CSkeletonLinkNew(void) {
}

CSkeletonLinkNew::~CSkeletonLinkNew(void) {
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
bool CSkeletonLinkNew::loadOsimBodySetObject(TiXmlElement* pSimmBodyElement) {
  //Load one osimBodySet object
  TiXmlElement *levelFiveChildElement, *levelSixChildElement,
      *levelSevenChildElement, *levelEightChildElement;
  bool loadFlag = true;
  const char *levelSixChildTag, *levelSevenChildTag;
  string sVTPString, sBuf;
  double temp0, temp1, temp2;//,*xyz_inertia;
  const char* str;

  CJointNew tmp_pJoint;

  // GET THE FIFTH LEVEL CHILD
  levelFiveChildElement = pSimmBodyElement;
  levelSixChildElement = levelFiveChildElement->FirstChildElement(); //Actual link data

  //Set name of link
  name = levelFiveChildElement->Attribute("name");

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelSixChildElement != NULL) {//Loop iterates over level6 child elements <mass>...<mass_center>...<inertia>
    loadFlag = false;
    levelSixChildTag = (levelSixChildElement->Value());
    if (strcmp(levelSixChildTag, "WrapObjectSet") == 0) {
      // GET THE SEVENTH LEVEL CHILD -- List of objects
      levelSevenChildElement = levelSixChildElement->FirstChildElement();// objects
      levelSevenChildTag = (levelSevenChildElement->Value());
      if ((NULL == levelSevenChildElement) || (strcmp(levelSevenChildTag,
          "objects") != 0)) {
        loadFlag = false;
      }
      else {
        loadFlag = loadOsimWrapObjectSetObjects(levelSevenChildElement);
      }
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "mass") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &mass);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "mass_center") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      mass_center.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "inertia_xx") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &inertia_xx);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "inertia_yy") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &inertia_yy);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "inertia_zz") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &inertia_zz);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "inertia_xy") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &inertia_xy);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "inertia_xz") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &inertia_xz);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "inertia_yz") == 0) {
      str = levelSixChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &inertia_yz);
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "Joint") == 0) {
      //Joint which connects this link to a parent
      levelSevenChildElement = levelSixChildElement->FirstChildElement();
      //if(levelSevenChildElement != NULL)
      if (name != "ground") {
        levelSevenChildTag = (levelSevenChildElement->Value());
        while (levelSevenChildElement != NULL) {//Parse all the CustomJoints
          if (strcmp(levelSevenChildTag, "CustomJoint") == 0) {
            tmp_pJoint.isCustomJoint = true;
            tmp_pJoint.isWeldJoint = false;
            loadFlag = tmp_pJoint.loadJoint(levelSevenChildElement);
            jointVector.push_back(tmp_pJoint);
            tmp_pJoint.coordObjVector.clear();
            tmp_pJoint.transformAxisVector.clear();
          }
          else if (strcmp(levelSevenChildTag, "WeldJoint") == 0) {
            tmp_pJoint.isWeldJoint = true;
            tmp_pJoint.isCustomJoint = false;
            loadFlag = tmp_pJoint.loadJoint(levelSevenChildElement);
            jointVector.push_back(tmp_pJoint);
            tmp_pJoint.coordObjVector.clear();
            tmp_pJoint.transformAxisVector.clear();
          }
          levelSevenChildElement = levelSevenChildElement->NextSiblingElement();
        }
      }
      loadFlag = true;
    }
    else if (strcmp(levelSixChildTag, "VisibleObject") == 0) {
      // GET THE SIXTH LEVEL CHILD -> <geometry_files> f1.vtp f2.vtp f3.vtp...</>
      levelSevenChildElement = levelSixChildElement->FirstChildElement(); //vtp files
      while (levelSevenChildElement != NULL) {
        levelSevenChildTag = (levelSevenChildElement->Value());
        if (strcmp(levelSevenChildTag, "geometry_files") == 0) {
          if (NULL != levelSevenChildElement->FirstChild()) {
            str = levelSevenChildElement->FirstChild()->Value(); //Note that this doesn't accept blank strings
            //Get the string containing the vtp files
            stringstream ss(str); // Insert the string into a stream
            while (ss >> sBuf) //Reads stream till whitespace
            {
              if ((sBuf != "") && (sBuf != "NULL")) {
                vtpGeometryFiles.push_back(sBuf);
              }
            }
          }
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

    //	                SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelSixChildElement = levelSixChildElement->NextSiblingElement();
  }
  // DONE
  //delete xyz_inertia;
  return true;
}

bool CSkeletonLinkNew::loadOsimWrapObjectSetObjects(
    TiXmlElement* pWrapObjectSetObjects) {//WrapCylinders and WrapEllipsoids
  TiXmlElement *levelSevenChildElement, *levelEightChildElement;
  bool loadFlag = false;
  const char *levelEightChildTag;

  CWrapCylinderNew pWrapCylinder;
  CWrapEllipsoidNew pWrapEllipsoid;

  // GET THE FIFTH LEVEL CHILD
  levelSevenChildElement = pWrapObjectSetObjects;
  levelEightChildElement = levelSevenChildElement->FirstChildElement(); //Actual links

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelEightChildElement != NULL) {//Loop iterates over level5 child elements <SimmBody>...
    levelEightChildTag = (levelEightChildElement->Value());
    if (strcmp(levelEightChildTag, "WrapCylinder") == 0) {
      //Read in a body link into the link vector
      pWrapCylinder.loadWrapCylinderObject(levelEightChildElement);
      wrapCylinderVector.push_back(pWrapCylinder);
      loadFlag = true;
    }
    else if (strcmp(levelEightChildTag, "WrapEllipsoid") == 0) {
      //Read in a body link into the link vector
      pWrapEllipsoid.loadWrapEllipsoidObject(levelEightChildElement);
      wrapEllipsoidVector.push_back(pWrapEllipsoid);
      loadFlag = true;
    }
    else {
      //SAIAssertSz( false, "Error: Invalid XML format in loaded osim model - failed to load." );
      //return false;
      loadFlag = true;
    }

    //                SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelEightChildElement = levelEightChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

}
