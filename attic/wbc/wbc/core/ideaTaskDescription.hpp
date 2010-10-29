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

#error this file is just a draft, do not use it

#ifndef WBC_TASK_DESCRIPTION_HPP
#define WBC_TASK_DESCRIPTION_HPP

#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <wbc/core/RobotControlModel.hpp>
#include <saimatrix/SAIQuaternion.h>

namespace wbc {

  class TaskDescription {
  public:
    TaskDescription(RobotControlModel * robmodel)
      : robmodel_(robmodel) {}
    
    virtual ~TaskDescription() {}

    virtual const SAIVector& commandAccel() const = 0;
    virtual const SAIMatrix& Jacobian() const = 0;
    virtual void onUpdate() = 0;

    virtual bool posConfig( SAIVector& )     const { return false; }
    virtual bool oriConfig( SAIQuaternion& ) const { return false; }
    virtual bool postureConfig( SAIVector& ) const { return false; }

    virtual bool goalPosConfig( SAIVector const & )     { return false; }
    virtual bool goalOriConfig( SAIQuaternion const & ) { return false; }
    virtual bool goalPostureConfig( SAIVector const & ) { return false; }
  
    virtual void propGain( double ) {}
    virtual void diffGain( double ) {}

    virtual void maxVel( double ) {}
    virtual void maxAccel( double ) {}

  protected:
    RobotControlModel * robmodel_;
  };

}

#endif // WBC_TASK_DESCRIPTION_HPP
