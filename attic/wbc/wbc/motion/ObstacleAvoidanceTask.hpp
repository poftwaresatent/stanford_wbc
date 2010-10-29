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
   \file ObstacleAvoidanceTask.hpp
   \author Duong (John) Dang
*/

#ifndef WBC_OBSTACLE_AVOIDANCE_TASK_HPP
#define WBC_OBSTACLE_AVOIDANCE_TASK_HPP

#include <wbc/core/TaskDescription.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <saimatrix/SAIVector.h>

namespace wbc {

  class ObstacleAvoidanceTask : public TaskDescription {
  public:
    explicit ObstacleAvoidanceTask(std::string const & name);
    ObstacleAvoidanceTask(std::string const & name, int nodeID,
			  SAIVector pointOnLink, SAIVector pointOnObstacle);
  
    virtual const SAIVector& commandAccel() const { return commandAccel_; }
    virtual const SAIMatrix& Jacobian() const { return globalJacobian_; }
  
    void cutoffDistance(double foo){cutoffDistance_ = foo;}
    void nodeIDs( int foo1, int foo2){ nodeID1_= foo1; nodeID2_=foo2;}
    void points( const SAIVector &foo1, const SAIVector &foo2){ point1_= foo1; point2_=foo2;}

    virtual void onUpdate(); 

    virtual void robotControlModel( RobotControlModel* robmodel ) throw(std::runtime_error)
    { robModel_ = robmodel; }
 
  private:
  
    // attributes
    SAIMatrix globalJacobian_;
    double distance_;
    double cutoffDistance_;
    SAIVector commandAccel_;
    int nodeID1_;
    int nodeID2_;
    SAIVector point1_;
    SAIVector point2_;
  
    // references
    RobotControlModel* robModel_;
  
    // private functions
    void servoUpdate(); 
  };

}

#endif
