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
  \file				Dynamics.cpp
*/
//==============================================================================

#include <wbc/core/Dynamics.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <tao/matrix/TaoDeMath.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/utility/TaoDeMassProp.h>
#include <cstdlib>

namespace wbc {

  Dynamics::Dynamics()
    : noj_( 0 ) {}


  void Dynamics::onUpdate( SAIVector const & jointAngles,
			   SAIVector const & jointVelocities ) 
  {
    curQ_ = jointAngles;
    curDQ_ = jointVelocities;
    
    // XXXX weirdness: this supposedly takes all joint angles and uses
    // them to compute the global translations and rotations of all
    // links. However, this is already done in Kinematics::onUpdate(),
    // which also first copies the joint state from the supplied
    // SAIVector instances into the TAO tree. So, probably we can just
    // delete this line.
    //
    // *EXCEPT THAT* inside computeCoriolisCentrifugalGravityForce(),
    // the joint positions are copied into the model AGAIN, but this
    // time setting the velocities to zero, and then it calls
    // taoDynamics::computeB() and taoDynamics::computeG(). Then, for
    // computeInvMassInertia(), it copies the full state again and
    // calls taoDynamics::fwdDynamics() and
    // taoDynamics::computeAinv(). Afterwards, inside
    // computeMassInertia(), it copies the same state yet again, and
    // calls taoDynamics::invDynamics() and taoDynamics::computeA().
    //
    // So, whatever. This looks like a ton of historic weirdness that
    // needs to be cleaned up from scratch, but should "only" hurt
    // performance (as opposed to the correctness of the
    // computations). I am leaving it all in place because it might
    // have some side-effects that are not apparent to me and that
    // make it work somehow.
    //
    // See http://sourceforge.net/apps/trac/stanford-wbc/ticket/44
    taoDynamics::updateTransformation(dynamicEvaluationModel2_->rootNode());
    
    computeCoriolisCentrifugalGravityForce();
    computeInvMassInertia();
    computeMassInertia();
  }


  void Dynamics::synchronize( SAIMatrix const & massInertia,
			      SAIMatrix const & invMassInertia,
			      SAIVector const & gravityForce,
			      SAIVector const & coriolisCentrifugalForce)
  {
    massInertia_ = massInertia;
    invMassInertia_ = invMassInertia;
    gravityForce_ = gravityForce;
    coriolisCentrifugalForce_ = coriolisCentrifugalForce;
  }


  void Dynamics::computeMassInertia() 
  {	
    for( int i = 0; i < noj_; i++ ) {

      taoDNode* node = dynamicEvaluationModel2_->idToNodeMap()[i];
      deFloat curQ; curQ = (deFloat) curQ_[i];
      deFloat curDQ; curDQ = (deFloat) 0.0;
      node->getJointList()[0].setQ( &curQ );	
      node->getJointList()[0].setDQ( &curDQ );
      node->getJointList()[0].zeroTau();
    }

    taoDynamics::invDynamics(dynamicEvaluationModel2_->rootNode(),&gravityAccel_);
    deFloat A[noj_*noj_];
    taoDynamics::computeA(dynamicEvaluationModel2_->rootNode(),noj_,A);

    for(int i = 0; i < noj_; i++)
      for(int j = 0; j < noj_; j++)
	massInertia_.elementAt(i,j) = A[i*noj_+j];
  }


  void Dynamics::computeInvMassInertia() 
  {
    for( int i = 0; i < noj_; i++ ) {

      taoDNode* node = dynamicEvaluationModel2_->idToNodeMap()[i];
      deFloat curQ; curQ = (deFloat) curQ_[i];
      deFloat curDQ; curDQ = (deFloat) 0.0;
      node->getJointList()[0].setQ( &curQ );	
      node->getJointList()[0].setDQ( &curDQ );
      node->getJointList()[0].zeroTau();
    }

    taoDynamics::fwdDynamics(dynamicEvaluationModel2_->rootNode(),&gravityAccel_);
    deFloat Ainv[noj_*noj_];
    taoDynamics::computeAinv(dynamicEvaluationModel2_->rootNode(),noj_,Ainv);

    for(int i = 0; i < noj_; i++)
      for(int j = 0; j < noj_; j++)
	invMassInertia_.elementAt(i,j) = Ainv[i*noj_+j];
  }


  void Dynamics::computeCoriolisCentrifugalGravityForce() 
  {

    for( int i = 0; i < noj_; i++ ) {

      taoDNode* node = dynamicEvaluationModel2_->idToNodeMap()[i];
      deFloat curQ; curQ = (deFloat) curQ_[i];
      deFloat curDQ; curDQ = (deFloat) 0.0;
      node->getJointList()[0].setQ( &curQ );	
      node->getJointList()[0].setDQ( &curDQ );
      node->getJointList()[0].zeroTau();
    }

    deFloat tmp[noj_];
    taoDynamics::computeB(dynamicEvaluationModel2_->rootNode(),noj_,tmp);
    for(int i = 0; i < noj_; i++) coriolisCentrifugalForce_[i] = tmp[i];

    taoDynamics::computeG(dynamicEvaluationModel2_->rootNode(),&gravityAccel_,noj_,tmp);
    for(int i = 0; i < noj_; i++) gravityForce_[i] = tmp[i];
  }


  void 
  Dynamics::robotControlModel( RobotControlModel* robotModel ) {

    // Get reference
    robotControlModel_ = robotModel;

    // Getting references
    dynamicEvaluationModel2_ = robotModel->branching();

    // Allocating variables
    noj_ = dynamicEvaluationModel2_->numJoints();
    massInertia_.setSize( noj_, noj_, true );
    invMassInertia_.setSize( noj_, noj_, true );
    gravityForce_.setSize( noj_, true );
    coriolisCentrifugalForce_.setSize( noj_, true );

    SAIVector g = robotModel->branching()->gravityAcceleration();
    gravityAccel_.set((deFloat) g[0], (deFloat) g[1], (deFloat) g[2]);
  }

}
