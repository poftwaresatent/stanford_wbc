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

//===========================================================================================
/*!
  \author			Luis Sentis
  \file				Kinematics.h
*/
//===========================================================================================

#ifndef WBC_KINEMATICS_H
#define WBC_KINEMATICS_H

#include <string>
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <saimatrix/SAITransform.h>
#include <wbc/core/BranchingRepresentation.hpp>

class taoDNode;

namespace wbc {

  class RobotControlModel;

  /*!
    \brief An entity that provides the physical whole body dynamic properties
    of a robot.
  */
  class Kinematics {

  public:

    Kinematics();
    ~Kinematics() {}

    const SAIVector & jointPositions() const { return curQ_;}
    const SAIVector & jointVelocities() const { return curDQ_;}

    static SAITransform globalFrame( taoDNode* node, const SAIVector & relativeDisplacement );
    
    /** This function returns the Jacobian at a global location. By
     * default, the function getJg from the tao library returns column
     * Jacobians with respect to the global frame. Therefore they need to
     * be displaced (not rotated since they are already in global frame)
     * to the desired point. Anecdotically, this function took me three
     * days to debug :) (Luis Sentis, lsentis@cs.stanford.edu).
     *
     * (Unfortunately we had to debug it some more : Samir and Marco!)
     *
     * If no global point is passed, this function will return the
     * Jacobian at the node's joint.
     */
    SAIMatrix JacobianAtPoint( taoDNode const * node, const SAIVector & globalPointOnLink );
  
    // read CoM and Jacobian CoM
    const SAIVector& COM() const { return com_; }
    const SAIMatrix& JacobianCOM() const { return Jcom_; }

    // read frame between feet joints
    const SAITransform& frameBetweenFeetJoints() const { return frameBetweenFeetJoints_; }

    /** Read sensors and update dynamic quantities.
     */
    void onUpdate( SAIVector const & jointAngles,
		   SAIVector const & jointVelocities );
  
    /**
       Checks the current joint angles against the upper and lower
       limits registered in BranchingRepresentation. If a non-null
       stream is provided, it also prints each violating joint.
     
       \return true if all joint angles are within limits, false if one
       of them lies outside.
    */
    bool checkDisplayJointLimits(std::ostream * os, char const * title, char const * prefix);

  protected:

    //! Pass rerefence methods
    void robotControlModel( RobotControlModel* );

  private:

    //! Attributes and copies
    int noj_;
    SAIVector com_;
    SAIMatrix Jcom_;
    SAIVector curQ_;
    SAIVector curDQ_;
    SAITransform frameBetweenFeetJoints_;

    //! Attributes -- references
    BranchingRepresentation* branchingModel_;
    RobotControlModel* robmodel_;	
    taoDNode * left_foot_;
    taoDNode * right_foot_;
    
    //! Private functions
    void calculateCOM();
    void calculateJacobianCOM();
    void calculateFrameBetweenFeetJoints();

    //! Friends
    friend class RobotControlModel;
  };

}

#endif // WBC_KINEMATICS_H
