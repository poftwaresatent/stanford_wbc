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
  \file       WholeBodyPosture.hpp
*/
//===========================================================================

#ifndef WBC_WHOLE_BODY_POSTURE_HPP
#define WBC_WHOLE_BODY_POSTURE_HPP

#include <wbc/core/TaskDescription.hpp>
#include <wbc/core/BranchingRepresentation.hpp>

namespace wbc {

  class WholeBodyPosture : public TaskDescription {
  public:

    explicit WholeBodyPosture(std::string const & name); 

    virtual const SAIVector& commandAccel() const { return commandAccel_; }
    virtual const SAIMatrix& Jacobian() const { return globalJacobian_; }
    virtual void goalPostureConfig( SAIVector goalConfig ) { goalConfig_ = goalConfig; }
    virtual const SAIVector& postureConfig() const { return config_; }
    virtual const SAIVector& goalPostureConfig() const { return goalConfig_; }
  
    virtual void onUpdate();

    virtual void robotControlModel( RobotControlModel* ) throw(std::runtime_error); 

    bool enable_vsat_;
    
  private:

    // attributes
    SAIVector velocity_;
    SAIMatrix globalJacobian_;
    SAIVector commandAccel_;
    SAIVector goalConfig_;
    SAIVector config_;
    SAIMatrix unactuationMatrix_;

    // references
    RobotControlModel* robModel_;
    
    // private functions
    void servoUpdate();
  };

}

#endif
