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
 * \file OrientationTask.hpp
 * \author Luis Sentis
 */

#ifndef WBC_Orientation_TASK_HPP
#define WBC_Orientation_TASK_HPP

#include <wbc/core/TaskDescription.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <saimatrix/SAIQuaternion.h>

namespace wbc {

  class OrientationTask : public TaskDescription {
  public:
    OrientationTask( std::string const & name,
		     std::string const & linkName );
    
    virtual const SAIVector& commandAccel() const { return commandAccel_; }
    virtual const SAIMatrix& Jacobian() const { return globalJacobian_; }
    virtual void goalOriConfig( SAIQuaternion goalConfig ) { goalConfig_ = goalConfig; }

    virtual const SAIQuaternion& oriConfig() const { return config_; }

    virtual const TaskType taskType() const {return Non_Contact_Task;} 

    virtual void directionSelection( bool xdir, bool ydir, bool zdir ) {}

    virtual void onUpdate(); 

    virtual void robotControlModel( RobotControlModel * robmodel ) throw(std::runtime_error);
    
  private:
  
    // attributes
    SAIVector velocity_;
    SAIMatrix globalJacobian_;
    SAIVector commandAccel_;
    SAIQuaternion goalConfig_;
    SAIQuaternion config_;
    std::string const linkName_;
    taoDNode * link_;
    
    // references
    RobotControlModel* robModel_;

    // private functions
    void servoUpdate(); 
  };

}

#endif
