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
 \file       MuscleDefinition.h (provides a template for a muscle)
 */
//=========================================================================
#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include "../OsimHeaders.hpp"

namespace osimparser {
using namespace std;
using namespace wbc_tinyxml;

struct SMusclePoint {
  string name;
  SAIVector3 location;
  string attached_to_link;
};

class CMuscleDefinition {
public:
  //Muscle identifiers
  string name;
  vector<SMusclePoint> vAttachedPoints;

  //Muscle characterizing properties (for the dynamical system)
  double muscle_model_type;
  double max_isometric_force, optimal_fiber_length, tendon_slack_length,
      pennation_angle;
  double activation_time_constant, deactivation_time_constant, Vmax, Vmax0;
  double FmaxTendonStrain, FmaxMuscleStrain, KshapeActive, KshapePassive,
      damping, Af, Flen;

  CMuscleDefinition(void);
  ~CMuscleDefinition(void);

  bool loadMuscleDef(TiXmlElement* pMuscleElement); //Loads an <Model><ActuatorSet><objects>...
};

}
