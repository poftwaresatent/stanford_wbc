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
 \file       MuscleDefinition.cpp (provides a template for a muscle)
 */
//=========================================================================

#include "CMuscleDefinition.h"

namespace osimparser {
using namespace std;

CMuscleDefinition::CMuscleDefinition(void) {
}

CMuscleDefinition::~CMuscleDefinition(void) {
}

//===========================================================================
//  - PUBLIC METHOD -
/*!
 Load an osim muscle model element into the class. This contains a
 description of a single link (ActuatorSet->objects->Thelen2003Muscle)

 \fn       bool loadMuscle( TiXmlElement* pMuscleElement )
 \param    pMuscleElement xml element containing a single Muscle object
 \return   Return true if file was loaded successfully
 */
//===========================================================================
bool CMuscleDefinition::loadMuscleDef(TiXmlElement* pMuscleElement) {
  TiXmlElement *levelThreeChildElement, *levelFourChildElement,
      *levelFiveChildElement;
  TiXmlElement *levelSixChildElement, *levelSevenChildElement;
  bool loadFlag = false;
  const char *levelFourChildTag, *levelSixChildTag;
  const char *str;
  double temp0, temp1, temp2;

  // get the second level child
  levelThreeChildElement = pMuscleElement;
  name = levelThreeChildElement->Attribute("name");

  levelFourChildElement = levelThreeChildElement->FirstChildElement();

  // Scan Muscle details
  while (levelFourChildElement != NULL) {//loop iterates over level4 child elements <MusclePointSet> ... <bodyset> ...<coordset>.. <jointset>
    levelFourChildTag = (levelFourChildElement->Value());
    if (strcmp(levelFourChildTag, "MusclePointSet") == 0) {
      // get the fourth level child -- list of muscle point objects
      levelFiveChildElement = levelFourChildElement->FirstChildElement(
          "objects");
      levelSixChildElement = levelFiveChildElement->FirstChildElement();
      while (levelSixChildElement != NULL) {
        SMusclePoint tmp_musclePts;
        levelSixChildTag = (levelSixChildElement->Value());
        if (strcmp(levelSixChildTag, "MusclePoint") == 0) {
          //Set muscle name
          if (NULL != levelSixChildElement->Attribute("name")) {
            tmp_musclePts.name = levelSixChildElement->Attribute("name");
          }
          else {
            tmp_musclePts.name = "NoName";
          }

          //Set muscle locations
          levelSevenChildElement = levelSixChildElement->FirstChildElement(
              "location");
          if (NULL == levelSevenChildElement) {
            loadFlag = false;
            //								SAIAssertSz( loadFlag, "Error: Invalid xml format in loaded muscle model at MusclePoint. No <location> tag." );
          }
          str = levelSevenChildElement->FirstChild()->Value();
          sscanf(str, "%lf %lf %lf", &temp0, &temp1, &temp2);
          tmp_musclePts.location.values(temp0, temp1, temp2);

          //Set body
          levelSevenChildElement = levelSixChildElement->FirstChildElement(
              "body");
          tmp_musclePts.attached_to_link
              = levelSevenChildElement->FirstChild()->Value();

          vAttachedPoints.push_back(tmp_musclePts);

          loadFlag = true;
        }

        // move to next sibling element
        levelSixChildElement = levelSixChildElement->NextSiblingElement();
      }//End of level 6 while loop
    }
    else if (strcmp(levelFourChildTag, "muscle_model") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      muscle_model_type = temp0;
    }
    else if (strcmp(levelFourChildTag, "max_isometric_force") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      max_isometric_force = temp0;
    }
    else if (strcmp(levelFourChildTag, "optimal_fiber_length") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      optimal_fiber_length = temp0;
    }
    else if (strcmp(levelFourChildTag, "tendon_slack_length") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      tendon_slack_length = temp0;
    }
    else if (strcmp(levelFourChildTag, "pennation_angle") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      pennation_angle = temp0;
    }
    else if (strcmp(levelFourChildTag, "activation_time_constant") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      activation_time_constant = temp0;
    }
    else if (strcmp(levelFourChildTag, "deactivation_time_constant") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      deactivation_time_constant = temp0;
    }
    else if (strcmp(levelFourChildTag, "Vmax") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      Vmax = temp0;
    }
    else if (strcmp(levelFourChildTag, "Vmax0") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      Vmax0 = temp0;
    }
    else if (strcmp(levelFourChildTag, "FmaxTendonStrain") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      FmaxTendonStrain = temp0;
    }
    else if (strcmp(levelFourChildTag, "FmaxMuscleStrain") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      FmaxMuscleStrain = temp0;
    }
    else if (strcmp(levelFourChildTag, "KshapeActive") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      KshapeActive = temp0;
    }
    else if (strcmp(levelFourChildTag, "KshapePassive") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      KshapePassive = temp0;
    }
    else if (strcmp(levelFourChildTag, "damping") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      damping = temp0;
    }
    else if (strcmp(levelFourChildTag, "Af") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      Af = temp0;
    }
    else if (strcmp(levelFourChildTag, "Flen") == 0) {
      str = levelFourChildElement->FirstChild()->Value();
      sscanf(str, "%lf", &temp0);
      Flen = temp0;
    }
    // move to next element
    levelFourChildElement = levelFourChildElement->NextSiblingElement();
  }

  // SAIAssertSz( loadFlag, "error: invalid xml format in loaded muscle model - failed to load." );

  // done
  return true;
}

}
