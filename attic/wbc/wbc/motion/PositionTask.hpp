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

//===========================================================================
/*!
  \author     Luis Sentis
  \file       PositionTask.hpp 
*/
//===========================================================================

#ifndef WBC_POSITION_TASK_HPP
#define WBC_POSITION_TASK_HPP

#include <wbc/core/TaskDescription.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>

namespace wbc {

  class PositionTask
    : public TaskDescription
  {
  public:
    PositionTask( std::string const & name,
		  std::string const & linkName );

    virtual const SAIVector& commandAccel() const { return commandAccel_; } 
    virtual const SAIMatrix& Jacobian() const { return globalJacobian_; }
    virtual void goalPosConfig( SAIVector goalConfig ) { goalConfig_ = goalConfig; }

    virtual const SAIVector& posConfig() const { return config_; }
    virtual const SAIVector& goalPosConfig() const { return goalConfig_; }
  
    // XXXX fuse this into generalized vectors of parameters
    void propGainVector( SAIVector propGainVec ) { propGainVec_ = propGainVec; }
    void diffGainVector( SAIVector diffGainVec ) { diffGainVec_ = diffGainVec; }

    virtual const TaskType taskType() const {return Non_Contact_Task;} 

    virtual void directionSelection( bool xdir, bool ydir, bool zdir );

    virtual void onUpdate(); 

    virtual void robotControlModel( RobotControlModel * robmodel ) throw(std::runtime_error);
 
  private:
  
    // attributes
    SAIVector velocity_;
    SAIMatrix globalJacobian_;
    SAIVector commandAccel_;
    SAIVector propGainVec_;
    SAIVector diffGainVec_;
    SAIVector goalConfig_;
    SAIVector config_;
    std::string const linkName_;
    taoDNode * link_;
    SAIMatrix selectionMatrix_;
  
    // references
    RobotControlModel* robModel_;

    // private functions
    void servoUpdate(); 
  };

}

#endif
