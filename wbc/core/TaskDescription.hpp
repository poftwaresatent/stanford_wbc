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

/**
 * \file TaskDescription.hpp
 * \author Luis Sentis
 */

#ifndef WBC_TASK_DESCRIPTION_HPP
#define WBC_TASK_DESCRIPTION_HPP

#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <saimatrix/SAIQuaternion.h>
#include <stdexcept>
#include <wbc/core/RobotControlModel.hpp>

#ifdef WIN32
#include "extras.h"
#endif

namespace wbc {

  class RobotControlModel;

  class TaskDescription {
  public:
  
    TaskDescription(){}
    TaskDescription(std::string const & _name,
		    double propGain, double diffGain,
		    double maxVel, double maxAccel);
  
    virtual ~TaskDescription() {}
  
    virtual const SAIVector& commandAccel() const;
    virtual const SAIMatrix& Jacobian() const;

    virtual const SAIVector& posConfig() const;
    virtual const SAIQuaternion& oriConfig() const;
    virtual const SAIVector& postureConfig() const;
    virtual const SAIVector& goalPosConfig() const;
    virtual const SAIQuaternion& goalOriConfig() const;
    virtual const SAIVector& goalPostureConfig() const;
    virtual void goalPosConfig( SAIVector const & ) {}
    virtual void goalOriConfig( SAIQuaternion const & ) {}
    virtual void goalPostureConfig( SAIVector const & ) {}
  
    inline void propGain( double gain )  { m_propGain = gain; }
    inline void intGain(  double gain )  { m_intGain = gain; }
    inline void diffGain( double gain )  { m_diffGain = gain; }
    inline void maxVel(   double vel )   { m_maxVel = vel;}
    inline void maxAccel( double accel ) { m_maxAccel = accel;}
  
    inline double propGain() const { return m_propGain; }
    inline double intGain()  const { return m_intGain; }
    inline double diffGain() const { return m_diffGain; }
    inline double maxVel()   const { return m_maxVel; }
    inline double maxAccel() const { return m_maxAccel; }
  
    typedef enum{ Contact_Task, Non_Contact_Task, Null_Task } TaskType;
    typedef enum{ Undefined_Task, Right_Foot_Contact_Task, Left_Foot_Contact_Task } TaskTag;
 
    virtual const TaskType taskType() const {return Null_Task;} 
    virtual const TaskTag taskTag() const {return Undefined_Task;}

    virtual void directionSelection( bool xdir, bool ydir, bool zdir ) {}

    virtual void onUpdate() {}
    
    /**
       Ends up being called from
       BehaviorDescription::loadMovementPrimitives(). For tasks that
       need special handling of the robot control model, this allows
       them to do so here, and throw an exception in case something is
       amiss. For example, a position task for a link name that does
       not exist in the robot can complain here.
    */
    virtual void robotControlModel( RobotControlModel* ) throw(std::runtime_error) {}
  
    std::string const name;
  
  protected:
    double m_propGain;
    double m_diffGain;
    double m_intGain; // XXXX defaults to zero, cannot be set at construction
    double m_maxVel;
    double m_maxAccel;
  };

}

#endif
