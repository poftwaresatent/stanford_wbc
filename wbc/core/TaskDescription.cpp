/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
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

#include <wbc/core/TaskDescription.hpp>

namespace {
  static SAIVector const nullVector;
  static SAIMatrix const nullMatrix;
  static SAIQuaternion const nullQuaternion;
}

namespace wbc {

  TaskDescription::
  TaskDescription(std::string const & _name,
		  double propGain, double diffGain,
		  double maxVel, double maxAccel)
    : name(_name),
      m_propGain(propGain),
      m_diffGain(diffGain),
      m_intGain(0),		// maybe make this configurable?
      m_maxVel(maxVel),
      m_maxAccel(maxAccel)
  {
  }
  
  
  SAIVector const & TaskDescription::
  commandAccel() const
  {
    return nullVector;
  }
  
  
  SAIMatrix const & TaskDescription::
  Jacobian() const
  {
    return nullMatrix;
  }


  SAIVector const & TaskDescription::
  posConfig() const
  {
    return nullVector;
  }


  SAIQuaternion const & TaskDescription::
  oriConfig() const
  {
    return nullQuaternion;
  }


  SAIVector const & TaskDescription::
  postureConfig() const
  {
    return nullVector;
  }


  SAIVector const & TaskDescription::
  goalPosConfig() const
  {
    return nullVector;
  }


  SAIQuaternion const & TaskDescription::
  goalOriConfig() const
  {
    return nullQuaternion;
  }


  SAIVector const & TaskDescription::
  goalPostureConfig() const
  {
    return nullVector;
  }

}
