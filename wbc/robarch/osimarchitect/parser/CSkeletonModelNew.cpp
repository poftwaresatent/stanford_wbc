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
 \file       SkeletonModel.cpp (stores all data obtained from a skeletal model)
 */
//=========================================================================

#include "CSkeletonModelNew.h"

namespace osimparser {

CSkeletonModelNew::CSkeletonModelNew(void) {
}

CSkeletonModelNew::~CSkeletonModelNew(void) {
}

//===========================================================================
//  - PUBLIC METHOD -
/*!
 Load an osim XML file into the class. The osim XML contains a complete description
 of the body to be included in the simulation.

 \fn       bool loadOsimFile( const char* osimFileName)
 \param    iFileName Name of file.
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CSkeletonModelNew::loadOsimFile(const char * osimFileName) {
  //LOAD XML FILE INTO MEMORY:
  //        char szFullPath[MAX_PATH];
  TiXmlElement* rootElement;
  TiXmlElement *levelZeroChildElement = NULL, *levelOneChildElement = NULL,
      *levelTwoChildElement = NULL;
  bool loadFlag = true;
  const char* rootTag, *levelZeroChildTag, *levelOneChildTag,
      *levelTwoChildTag, *levelOneChildText;

  //CDirRestorer dirChange( osimFileName, szFullPath );
  if (true != osimDocument.LoadFile(osimFileName)) {
    //                SAIAssertSz( false, "Error in osim XML file - failed to load." );
    return false;
  }

  //SCAN FOR ROOT ELEMENT
  rootElement = osimDocument.RootElement();

  // FOUND TAG "WORLD":
  rootTag = (rootElement->Value());

  if (strcmp(rootTag, "OpenSimDocument") != 0) {
    //Uncomment following lines for rigidly following new format
    //SAIAssertSz( false, "Error: Invalid XML format in loaded osim model - failed to load." );
    //return false;

    //Remove following lines once you are sure old format will not be mixed with the new one
    if (strcmp(rootTag, "Model") == 0) {
      //SAIAssertSz( false, "Error: Invalid XML format in loaded osim model - failed to load." );
      //return false;
      levelZeroChildElement = rootElement;
    }
  }
  else {
    //Get the model
    levelZeroChildElement = rootElement->FirstChildElement();
    // FOUND TAG "WORLD":
    levelZeroChildTag = (levelZeroChildElement->Value());

    if (strcmp(levelZeroChildTag, "Model") != 0) {
      //					SAIAssertSz( false, "Error: Invalid XML format in loaded osim model - failed to load." );
      return false;
    }
  }

  // GET THE FIRST CHILD
  levelOneChildElement = levelZeroChildElement->FirstChildElement();

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelOneChildElement != NULL) {
    levelOneChildTag = (levelOneChildElement->Value());
    loadFlag = false;
    if (strcmp(levelOneChildTag, "defaults") == 0) {
      //Add handler for case: Schutte1993Muscle etc.
      //This will include all the muscle data
      //Not relevant for motion reconstruction in SAI since we only do
      //a trajectory task and generate joint angles which are then fed
      //back into opensim.
      //Phase 2:
      //This module will become relevant when we need to write controllers
      //for the muscles.
      loadFlag = true;
    }
    else if (strcmp(levelOneChildTag, "angle_units") == 0) {
      //Add handler for setting angle_units
      levelOneChildText = levelOneChildElement->FirstChild()->Value();
      angle_units = levelOneChildText;
      loadFlag = true;
    }
    else if (strcmp(levelOneChildTag, "length_units") == 0) {
      //Add handler for setting angle_units
      levelOneChildText = levelOneChildElement->FirstChild()->Value();
      length_units = levelOneChildText;
      loadFlag = true;
    }
    else if (strcmp(levelOneChildTag, "force_units") == 0) {
      //Add handler for setting angle_units
      levelOneChildText = levelOneChildElement->FirstChild()->Value();
      force_units = levelOneChildText;
      loadFlag = true;
    }
    else if (strcmp(levelOneChildTag, "DynamicsEngine") == 0) {
      //Main osim data is loaded here
      levelTwoChildElement = levelOneChildElement->FirstChildElement();//SimmKinematicsEngine
      levelTwoChildTag = (levelTwoChildElement->Value());
      if ((NULL == levelTwoChildElement) || (strcmp(levelTwoChildTag,
          "SimbodyEngine") != 0)) {
        loadFlag = false;
      }
      else {
        loadFlag = loadSimbodyEngine(levelTwoChildElement);
      }
    }
    else if (strcmp(levelOneChildTag, "ActuatorSet") == 0) {
      //Add handler for case: This will include all the muscle data
      levelTwoChildElement = levelOneChildElement->FirstChildElement("objects");//SimmKinematicsEngine
      levelTwoChildTag = (levelTwoChildElement->Value());
      if ((NULL == levelTwoChildElement)
          || (strcmp(levelTwoChildTag, "objects") != 0)) {
        loadFlag = false;
      }
      else {
        loadFlag = loadMuscleObjects(levelTwoChildElement);
      }
      //SAIAssertSz( loadFlag, "Error in osim model : Failed to load ActuatorSet." );
    }
    else {
      //NOTE TODO: Set to false if the above list of options is exhaustive
      loadFlag = true;
    }

    //                SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelOneChildElement = levelOneChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

//===========================================================================
//  - PRIVATE METHOD -
/*!
 Load an osim Simbody xml element into the class. This contains a
 description of the links (BodySet->objects->SimmBody), the degrees of fredom
 (CoordinateSet->objects->SimmCoordinate), and the joint data
 (JointSet->objects->SimmJoint)

 \fn       bool loadSimbodyEngine(const TiXmlElement * simmKinematicsEngineElement)
 \param    iFileName Name of file.
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CSkeletonModelNew::loadSimbodyEngine(TiXmlElement * simbodyEngineElement) {
  TiXmlElement *levelTwoChildElement, *levelThreeChildElement,
      *levelFourChildElement;
  bool loadFlag = false;
  const char *levelThreeChildTag, *levelFourChildTag;
  const char *str;
  double temp0, temp1, temp2;

  // GET THE SECOND LEVEL CHILD
  levelTwoChildElement = simbodyEngineElement;
  levelThreeChildElement = levelTwoChildElement->FirstChildElement();

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelThreeChildElement != NULL) {//Loop iterates over level3 child elements <gravity> ... <bodyset> ...<CoordSet>.. <JointSet>
    loadFlag = false;
    levelThreeChildTag = (levelThreeChildElement->Value());
    if (strcmp(levelThreeChildTag, "gravity") == 0) {
      //Add handler for case: gravity string = "x=0 y=9.8 z=0"
      str = levelThreeChildElement->FirstChild()->Value();
      sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
      gravity.values(temp0, temp1, temp2);
      loadFlag = true;
    }
    else if (strcmp(levelThreeChildTag, "BodySet") == 0) {
      // GET THE FOURTH LEVEL CHILD -- List of objects
      levelFourChildElement = levelThreeChildElement->FirstChildElement(
          "objects");
      levelFourChildTag = (levelFourChildElement->Value());
      if ((NULL == levelFourChildElement) || (strcmp(levelFourChildTag,
          "objects") != 0)) {
        loadFlag = false;
      }
      else {
        loadFlag = loadBodySetObjects(levelFourChildElement);
      }
    }
    else if (strcmp(levelThreeChildTag, "ConstraintSet") == 0) {
      //NOTE: TODO: IMPLEMENT THIS
      loadFlag = true;
    }
    else if (strcmp(levelThreeChildTag, "MarkerSet") == 0) {
      // GET THE FOURTH LEVEL CHILD -- List of objects
      levelFourChildElement = levelThreeChildElement->FirstChildElement();// objects
      levelFourChildTag = (levelFourChildElement->Value());
      if ((NULL == levelFourChildElement) || (strcmp(levelFourChildTag,
          "objects") != 0)) {
        loadFlag = false;
      }
      else {
        loadFlag = loadMarkerSetObjects(levelFourChildElement);
      }
    }
    else {
      //Replace with loadflag = false when exhaustive list of xml has been complied
      loadFlag = true;
    }

    //                assert( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelThreeChildElement = levelThreeChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

//===========================================================================
//  - PRIVATE METHOD -
/*!
 Load an osim SimmKinematicsEngine xml element into the class. This contains a
 description of the links (BodySet->objects->SimmBody)

 \fn       bool loadBodySetObjects(TiXmlElement * bodySetObjectElement)
 \param    bodySetObjectElement xml element containing a set of SimmBody objects
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CSkeletonModelNew::loadBodySetObjects(TiXmlElement * bodySetObjectElement) {
  TiXmlElement *levelFourChildElement, *levelFiveChildElement;
  bool loadFlag = false;
  const char *levelFiveChildTag;
  CSkeletonLinkNew pSimmBodyLink;

  // GET THE FIFTH LEVEL CHILD
  levelFourChildElement = bodySetObjectElement;
  levelFiveChildElement = levelFourChildElement->FirstChildElement(); //Actual links

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelFiveChildElement != NULL) {//Loop iterates over level5 child elements <Body>...
    levelFiveChildTag = (levelFiveChildElement->Value());
    if (strcmp(levelFiveChildTag, "Body") == 0) {
      //Read in a body link into the link vector
      pSimmBodyLink.loadOsimBodySetObject(levelFiveChildElement);

      //Find the id of the link and its parent
      if (vSimmBodyVector.begin() == vSimmBodyVector.end()) //First node--typically "ground"
      {
        pSimmBodyLink.parent_id = -1;
        pSimmBodyLink.self_id = -1;
      }
      else {
        if (pSimmBodyLink.jointVector.begin()
            == pSimmBodyLink.jointVector.end()) {//Has no parent node (This should ideally not execute since links are attached to the ground
          pSimmBodyLink.parent_id = -1;
          pSimmBodyLink.self_id = -1;
        }
      }
      //SAIAssertSz(pSimmBodyLink->parent_id >= 0, "Error: Disconnected Link");

      //Add the link to the SimmBody vector (aka. the link vector)
      pSimmBodyLink.self_id = vSimmBodyVector.size() - 1;
      vSimmBodyVector.push_back(pSimmBodyLink);
      pSimmBodyLink.wrapCylinderVector.clear();
      pSimmBodyLink.wrapEllipsoidVector.clear();
      pSimmBodyLink.jointVector.clear();
      loadFlag = true;
    }
    else {
      //SAIAssertSz( false, "Error: Invalid XML format in loaded osim model - failed to load." );
      return false;
    }

    //SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelFiveChildElement = levelFiveChildElement->NextSiblingElement();
  }
  //Assign parent ids to all nodes.
  vector<osimparser::CSkeletonLinkNew>::iterator it, it_end;
  it_end = vSimmBodyVector.end();
  vector<CJointNew>::iterator skeleJoint;
  for (it = vSimmBodyVector.begin(); it != it_end; ++it) {
    if (it->jointVector.begin() != it->jointVector.end()) {
      skeleJoint = it->jointVector.begin();
      it->parent_id = findLinkId((*skeleJoint).parent_body);
    }
  }
  // DONE
  return true;
}

//===========================================================================
//  - PRIVATE METHOD -
/*!
 Load an osim SimmKinematicsEngine xml element into the class. This contains a
 description of the links (BodySet->objects->SimmBody)

 \fn       bool loadJointSetObjects(TiXmlElement * jointSetObjectElement)
 \param    jointSetObjectElement xml element containing a set of SimmBody objects
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CSkeletonModelNew::loadConstraintSetObjects(
    TiXmlElement * constraintSetObjectElement) {
  TiXmlElement *levelFourChildElement, *levelFiveChildElement;
  const char *levelFiveChildTag;
  //CSkeletonJoint pSimmJoint;

  // GET THE FIFTH LEVEL CHILD
  levelFourChildElement = constraintSetObjectElement;//Object containing all joints
  levelFiveChildElement = levelFourChildElement->FirstChildElement(); //One joint

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelFiveChildElement != NULL) {//Loop iterates over level5 child elements object-><Level5element>...
    levelFiveChildTag = (levelFiveChildElement->Value());

    //Write code to parse the "DynamicsEngine->SimbodyEngine->ConstraintSet" node
    //if (strcmp( levelFiveChildTag, "SimmJoint" ) == 0)
    //{
    //      //Read in a body link into the link vector
    //      pSimmJoint = new CSkeletonJoint();
    //      pSimmJoint->loadOsimJointSetObject(levelFiveChildElement);
    //      vSimmJointVector.push_back(*pSimmJoint);
    //      loadFlag = true;
    //}
    //else
    //{
    //      SAIAssertSz( false, "Error: Invalid XML format in loaded osim model - failed to load." );
    //      return false;
    //}
    //
    //SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelFiveChildElement = levelFiveChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

//===========================================================================
//  - PRIVATE METHOD -
/*!
 Load an osim SimmKinematicsEngine xml element into the class. This contains a
 description of the links (BodySet->objects->SimmBody)

 \fn       bool loadMarkerSetObjects(TiXmlElement * markerSetObjectElement)
 \param    markerSetObjectElement xml element containing a set of SimmBody objects
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CSkeletonModelNew::loadMarkerSetObjects(
    TiXmlElement * markerSetObjectElement) {
  TiXmlElement *levelFourChildElement, *levelFiveChildElement;
  bool loadFlag = false;
  const char *levelFiveChildTag;
  CSkeletonMarker pSimmMarker;

  // GET THE FIFTH LEVEL CHILD
  levelFourChildElement = markerSetObjectElement;//Object containing all markers
  levelFiveChildElement = levelFourChildElement->FirstChildElement(); //One marker

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelFiveChildElement != NULL) {//Loop iterates over level5 child elements object-><Marker>...
    levelFiveChildTag = (levelFiveChildElement->Value());
    if (strcmp(levelFiveChildTag, "Marker") == 0) {
      //Read in a marker into the marker vector
      //NOTE TODO: Fix this --> implement a reset function instead of deleting and realloc
      pSimmMarker.loadOsimMarkerSetObject(levelFiveChildElement);
      vSimmMarkerVector.push_back(pSimmMarker);
      pSimmMarker.vtpGeometryFiles.clear();
      loadFlag = true;
    }
    else {
      //                        assert( false, "Error: Invalid XML format in loaded osim model - failed to load." );
      return false;
    }

    //                assert( loadFlag, "Error: Invalid XML format in loaded osim model - failed to load." );

    // MOVE TO NEXT ELEMENT
    levelFiveChildElement = levelFiveChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

int CSkeletonModelNew::findLinkId(string link_name) {
  //Find the id of the passed link
  int i = 0;
  for (vector<CSkeletonLinkNew>::iterator skeleLink = vSimmBodyVector.begin(); skeleLink
      != vSimmBodyVector.end(); skeleLink++) {
    if (skeleLink->name == link_name) {//found link
      return skeleLink->self_id; //return its id
    }
    i++; //Id of next link
  }
  return -2;
}

//===========================================================================
//  - PRIVATE METHOD -
/*!
 Load an osim Simbody xml element into the class. This contains a
 description of the links (ActuatorSet->objects->ThelenMuscle etc.)

 \fn       bool loadMuscleObjects(TiXmlElement * simbodyEngineElement)
 \param    TiXmlElement * Simbody Element.
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CSkeletonModelNew::loadMuscleObjects(TiXmlElement * simbodyEngineElement) //Level 2 passed
{
  TiXmlElement *levelTwoChildElement, *levelThreeChildElement;//// *levelFourChildElement;
  bool loadFlag = false;
  const char *levelThreeChildTag;//// *levelFourChildTag;
  ////const char *str;
  ////double temp0, temp1, temp2;

  // GET THE SECOND LEVEL CHILD
  levelTwoChildElement = simbodyEngineElement;
  levelThreeChildElement = levelTwoChildElement->FirstChildElement();

  // SCAN ALL SIBLING ELEMENTS IN GIVEN CONTAINER
  while (levelThreeChildElement != NULL) {//Loop iterates over level3 child elements <gravity> ... <bodyset> ...<CoordSet>.. <JointSet>
    loadFlag = false;
    levelThreeChildTag = (levelThreeChildElement->Value());
    if (strcmp(levelThreeChildTag, "Thelen2003Muscle") == 0) {
      CMuscleDefinition tmpMuscleDef;
      //GET THE FOURTH LEVEL CHILD -- List of objects
      loadFlag = tmpMuscleDef.loadMuscleDef(levelThreeChildElement);
      if (loadFlag == true) {
        vMuscleDefVector.push_back(tmpMuscleDef);
      }
      //						else
      //						{	SAIAssertSz( loadFlag, "Error: Invalid XML in osim model - Bad Thelen2003Muscle Muscle Definition." );	}
    }
    else if (strcmp(levelThreeChildTag, "Schutte1993Muscle") == 0) {
      CMuscleDefinition tmpMuscleDef;
      //GET THE FOURTH LEVEL CHILD -- List of objects
      loadFlag = tmpMuscleDef.loadMuscleDef(levelThreeChildElement);
      if (loadFlag == true) {
        vMuscleDefVector.push_back(tmpMuscleDef);
      }
      //						else
      //						{	SAIAssertSz( loadFlag, "Error: Invalid XML in osim model - Bad Schutte1993Muscle Muscle Definition." );	}
    }
    //                SAIAssertSz( loadFlag, "Error: Invalid XML format in loaded osim model - Failed to load muscle models." );

    // MOVE TO NEXT ELEMENT
    levelThreeChildElement = levelThreeChildElement->NextSiblingElement();
  }
  // DONE
  return true;
}

}
