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

//==============================================================================
/*!
  \author			Luis Sentis
  \file				Dynamics.h
*/
//==============================================================================

#ifndef WBC_DYNAMICS_H
#define WBC_DYNAMICS_H

#include <saimatrix/SAIMatrix.h>
#include <saimatrix/SAIVector.h>
#include <tao/matrix/TaoDeMath.h>	

class taoNodeRoot;
class taoDNode;
class taoNode;

namespace wbc {

  class BranchingRepresentation;
  class RobotControlModel;

  /*!
    \brief An entity that provides the physical whole body dynamic properties
    of a robot.
  */

  class Dynamics {

  public:

    //! Constructor.
    Dynamics();
    ~Dynamics() {}

    //! Read mass-inertia.
    const SAIMatrix& massInertia() { return massInertia_; }

    //! Read inverse mass-inertia.
    const SAIMatrix& invMassInertia() { return invMassInertia_; }

    //! Read Coriolis-Centrifugal force.
    const SAIVector& coriolisCentrifugalForce() { return coriolisCentrifugalForce_; }

    //! Read gravity force.
    const SAIVector& gravityForce() { return gravityForce_; }

    /** Read sensors and update dynamic quantities.
     */
    void onUpdate( SAIVector const & jointAngles,
		   SAIVector const & jointVelocities );

    //! copy dynamic quantities if calculated in another process
    void synchronize( SAIMatrix const & massInertia,
		      SAIMatrix const & invMassInertia,
		      SAIVector const & gravityForce,
		      SAIVector const & coriolisCentrifugalForce );
    
    /** Get reference and initialize
     */
    void robotControlModel( RobotControlModel* );
    
  private:

    //! Attributes
    SAIMatrix massInertia_;
    SAIMatrix invMassInertia_;
    SAIVector coriolisCentrifugalForce_;
    SAIVector gravityForce_;
    int computeFlags_;
    SAIVector zeroMomentPoint_;
    deVector3 gravityAccel_;

    //! Attributes -- references
    BranchingRepresentation* dynamicEvaluationModel2_;
    RobotControlModel* robotControlModel_;	

    //! Attributes -- copies
    int noj_;
    SAIVector curQ_;
    SAIVector curDQ_;

    /** Computes the joint mass inertia matrix using recursive inverse dynamics. See
     * the thesis of Kyong-Sok Chang <kcchang AT cs.stanford.edu> for further details.
     */
    void computeMassInertia();
    
    /** Computes the inverse joint mass inertia matrix using recursive forward
     * dynamics. See the thesis of Kyong-Sok Chang <kcchang AT cs.stanford.edu> for
     * further details.
     */
    void computeInvMassInertia();
    
    /** Computes the combined joint Coriolis-Centrifugal and gravity forces using
     * recursive inverse dynamics. See the thesis of Kyong-Sok Chang
     * <kcchang AT cs.stanford.edu> for further details.
     */
    void computeCoriolisCentrifugalGravityForce();
    
    void computeZeroMomentPoint();

    // Friends
    friend class RobotControlModel;
  };

}

#endif // WBC_DYNAMICS_H
