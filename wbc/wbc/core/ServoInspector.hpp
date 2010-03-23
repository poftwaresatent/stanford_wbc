/*
 * Stanford_WBC_Extension -- plugin library for stanford-wbc.sourceforge.net
 *
 * Copyright (C) 2008, 2009 Stanford University
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

#ifndef WBC_SERVO_INSPECTOR_HPP
#define WBC_SERVO_INSPECTOR_HPP

#include <saimatrix/SAIVector.h>
#include <vector>

namespace wbc {
  
  class RobotControlModel;
  
  /**
     Container for centers of pressure information.
   */
  struct cop_data {
    cop_data() {}
    inline explicit cop_data(size_t nreserve) {
      linkID.reserve(nreserve);
      cop.reserve(nreserve);
    }
    
    std::vector<int> linkID;
    std::vector<SAIVector> cop;
  };
  
  
  /**
     Interface for breaking through some of the encapsulation of the
     whole-body controller... in some places, we simply need some
     whitebox information. For example, to display the centers of
     pressure in simulation while debugging contact behaviors.
  */
  class ServoInspector
  {
  public:
    virtual ~ServoInspector() {}
    virtual RobotControlModel * getRobotControlModel() = 0;
    
    /** If there are no contacts, the cop_data will have empty vectors. */
    virtual cop_data getCentersOfPressure() = 0;
    
    /** The center of mass is a 3D vector (barring bugs, of course). */
    virtual SAIVector getCOM() = 0;
    
    /** If we are not in contact, the zmp will be a zero-sized
	vector. Otherwise a 3D vector. Anything else is a bug. */
    virtual SAIVector getZMP() = 0;
  };
  
}

#endif // WBC_SERVO_INSPECTOR_HPP
