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
  \file       JointTask.hpp
*/
//===========================================================================

#ifndef WBC_JOINT_TASK_HPP
#define WBC_JOINT_TASK_HPP

#include <wbc/core/TaskDescription.hpp>
#include <wbc/core/BranchingRepresentation.hpp>

namespace wbc {

  class JointTask : public TaskDescription {
  public:

    explicit JointTask(std::string const & name);
  
    virtual const SAIVector& commandAccel() const { return commandAccel_; }
    virtual const SAIMatrix& Jacobian() const { return globalJacobian_; }
    virtual void goalPostureConfig( SAIVector const & goalConfig ) { goalConfig_ = goalConfig; }
    virtual SAIVector const & goalPostureConfig() const { return goalConfig_; }
    virtual const SAIVector& postureConfig() const { return config_; }

    void addJoint( int jointID );
  
    virtual const TaskType taskType() const { return Non_Contact_Task; } 

    virtual void onUpdate();

    virtual void robotControlModel( RobotControlModel* ) throw(std::runtime_error);

  private:

    // attributes
    SAIVector velocity_;
    SAIMatrix globalJacobian_;
    SAIVector commandAccel_;
    SAIVector goalConfig_;
    SAIVector config_;

    // references
    RobotControlModel* robModel_;
    SAIMatrix selection_matrix_;
    
    // private functions
    void servoUpdate();
    void addJoint();
  };

}

#endif
