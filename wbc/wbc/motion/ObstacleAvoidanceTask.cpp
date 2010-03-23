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
   \file ObstacleAvoidanceTask.cpp
   \author Duong (John) Dang
*/

#include <wbc/motion/ObstacleAvoidanceTask.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/Kinematics.hpp>
#include <saimatrix/SAITransform.h>
#include <sstream>
#include <iostream>
#include <limits>
#include <cstdlib>

namespace wbc {

  void ObstacleAvoidanceTask::onUpdate() {
    // update jacobian
    if ((!point1_) || (!point2_))
      distance_ = 2 * cutoffDistance_;
	else
      distance_ = (point1_ - point2_).length();

    if (distance_ > cutoffDistance_)
      globalJacobian_ = SAIMatrix(0,0);
    else {
      SAIMatrix J1, J2;
      if (nodeID1_!=-1){      
	taoDNode* node1 = robModel_->branching()->idToNodeMap()[nodeID1_];
	J1= robModel_->kinematics()->JacobianAtPoint( node1, point1_ );
      }
      else 
	J1 = SAIMatrix(6,robModel_->branching()->numJoints());
    
      if (nodeID2_!=-1){      
	taoDNode* node2 = robModel_->branching()->idToNodeMap()[nodeID2_];
	J2= robModel_->kinematics()->JacobianAtPoint( node2, point2_ );
      }
      else 
	J2 = SAIMatrix(6,robModel_->branching()->numJoints());
      SAIMatrix J=J1-J2;
      int col = J.column();
      J = J.submatrix( 0, 0, 3, col );

      SAIVector e12=(point1_ - point2_)/((point1_ - point2_).magnitude());
      SAIMatrix e12t(1,3);
      e12t.setSubvector(0,0,e12,true);

      globalJacobian_=e12t*J;
    }
    // update servo
    servoUpdate();
  }

  void ObstacleAvoidanceTask::servoUpdate() {
    commandAccel_=SAIVector(1);
    if (distance_ > cutoffDistance_)
      commandAccel_= SAIVector(3);
    else      
      {
	commandAccel_[0]
	  = m_propGain* (1/(distance_*distance_) - 1/(cutoffDistance_*cutoffDistance_));
      }
  }

  ObstacleAvoidanceTask::ObstacleAvoidanceTask(std::string const & name)
    : TaskDescription(name, 100, -1, -1, -1),
      distance_(1e99),
      cutoffDistance_(0.3),
      point1_(0),
      point2_(0)
  {
  }

}
