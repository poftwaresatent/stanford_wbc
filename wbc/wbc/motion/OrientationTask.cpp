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
 * \file OrientationTask.cpp
 * \author Luis Sentis
 */

#include <wbc/motion/OrientationTask.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/Kinematics.hpp>
#include <saimatrix/SAITransform.h>
#include <iostream> 

namespace wbc {

  OrientationTask::OrientationTask( std::string const & name,
				    std::string const & linkName ) 
    : TaskDescription(name, 400, 40, 0.6, 0),
      linkName_(linkName) {}


  void OrientationTask::onUpdate() {

    // update position
    config_ = robModel_->kinematics()->globalFrame( link_, SAIVector(0) ).rotation();

    // update jacobian
    globalJacobian_ = robModel_->kinematics()->JacobianAtPoint( link_, SAIVector(0) );
    int col = globalJacobian_.column();
    globalJacobian_ = globalJacobian_.submatrix( 3, 0, 3, col );
  
    // update velocity
    velocity_ = globalJacobian_ * robModel_->kinematics()->jointVelocities();
 
    // update servo
    servoUpdate();
  }


  void OrientationTask::servoUpdate() {
  
    SAIVector angularError;
    config_.angularError( goalConfig_, angularError );

    // desDiff = -( state_ - goalState_ ) * gain_
    SAIVector desDiff = -angularError * m_propGain;


    // Saturating upon velocity limits
    if( m_diffGain != 0 ) {
      double satFactor = desDiff.abs() / ( m_maxVel * m_diffGain );
      if( satFactor > 1 ) desDiff.multiply( 1/satFactor );
    }

    // desiredAccel_ = -m_diffGain (diffState_ - \nu desDiff_ );
    //cout << "desdiff size: " << desDiff.size() << "velocity size: " << velocity_.size() << std::endl;
    commandAccel_ = desDiff - velocity_ * m_diffGain; 

    //angularError.display("angular error");
    // Saturation of acceleration.
    double satFactor2, inv_rate;
    if( fabs(m_maxAccel) > 1e-2 ) { // this is to ensure that acceleration saturation has been set up
      inv_rate =  commandAccel_.abs() / m_maxAccel ;
      if( inv_rate > 1 ) satFactor2 = 1.0 / inv_rate;
      else satFactor2 = 1.0;
      commandAccel_ *= satFactor2;
    } 
  }
  
  
  void OrientationTask::
  robotControlModel( RobotControlModel * robmodel )
    throw(std::runtime_error)
  {
    robModel_ = robmodel;
    link_ = robmodel->branching()->findLink(linkName_);
    if ( ! link_)
      throw std::runtime_error("wbc::OrientationTask::robotControlModel(): no link with name `"
			       + linkName_ + "'");
  }

}
