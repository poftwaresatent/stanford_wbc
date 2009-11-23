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
  \file				Kinematics.cpp
*/
//==============================================================================
   
#include <wbc/core/Kinematics.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <tao/matrix/TaoDeMath.h>	
#include <tao/matrix/TaoDeQuaternion.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoTypes.h>
#include <tao/dynamics/taoJoint.h>


namespace {
  static SAIVector const nullVector;
}


namespace wbc {

  Kinematics::Kinematics() 
    : noj_( 0 ), com_(3), left_foot_(0), right_foot_(0) { 
  }


  void 
  Kinematics::calculateFrameBetweenFeetJoints() {
    frameBetweenFeetJoints_.identity();
    if (left_foot_ && right_foot_) {
      SAITransform xrf, xlf;
      SAIQuaternion phirf, philf;
      
      xrf = globalFrame( right_foot_, nullVector);
      xlf = globalFrame( left_foot_, nullVector);

      // subtract depth
      // XXXX to do: can cache the depths, instead of std::map lookup each time
      xrf.translation()[2] -= branchingModel_->surfaceDepth( right_foot_ );
      xlf.translation()[2] -= branchingModel_->surfaceDepth( left_foot_ );

      frameBetweenFeetJoints_.translation() = 0.5 * ( xrf.translation() + xlf.translation() );
      phirf = xrf.rotation();
      philf = xlf.rotation();
      frameBetweenFeetJoints_.rotation() = ( phirf + philf ) / 2.0;
    }
  }


  void Kinematics::calculateCOM() {

    deVector3 linkfoo, comglobal;
    taoDNode* node; 
    double totalMass = branchingModel_->totalMass();
    deQuaternion taoquat;

    com_.zero();
    comglobal.zero();
    linkfoo.zero();

    for( int ii(0); ii < branchingModel_->numJoints(); ++ii) {

      node = branchingModel_->idToNodeMap()[ii];

      // Get rotation
      taoquat = node->frameGlobal()->rotation();
      deMatrix3 rotmat; rotmat.set( taoquat );
      linkfoo.multiply( taoquat, *node->center() );
      deVector3 globalpos = node->frameGlobal()->translation();
      linkfoo += globalpos;
      linkfoo *= *node->mass();
      linkfoo *= (deFloat) (1.0 / totalMass);
      comglobal += linkfoo;
    }

    for( int ii(0); ii < 3; ++ii )
      com_[ii] = comglobal.elementAt(ii);
  }


  void Kinematics::calculateJacobianCOM() {

    // Jcom = 1/M \sum (m_i * J_comi)
    // J_comi = JacobianAtPonint(node, linkcomglobal)
    deVector3 linkcomglobal;
    taoDNode* node; 
    deFloat totalMass = branchingModel_->totalMass();
    SAIMatrix Jlinkcomglobal(3,noj_), Jcomglobal(3,noj_);
    deQuaternion taoquat;

    linkcomglobal.zero();
    Jlinkcomglobal.zero();
    Jcomglobal.zero();
  
    SAIVector linkcomglobal2(3);

    for( int ii(0); ii < branchingModel_->numJoints(); ++ii) {

      node = branchingModel_->idToNodeMap()[ii];

      taoquat = node->frameGlobal()->rotation();
      linkcomglobal.multiply( taoquat, *node->center() );
      linkcomglobal += node->frameGlobal()->translation();

      // translate to SAIVector
      for( int jj(0); jj < 3; ++jj)
	linkcomglobal2[jj] = linkcomglobal[jj];
      Jlinkcomglobal = JacobianAtPoint( node, linkcomglobal2 ).submatrix(0,0,3,noj_);
      Jlinkcomglobal *= (*node->mass() / totalMass);
      Jcomglobal += Jlinkcomglobal;
    }

    for( int ii(0); ii < 3; ++ii )
      for( int jj(0); jj < noj_; ++jj )
	Jcom_.elementAt(ii,jj) = Jcomglobal.elementAt(ii,jj);
  }
  
  
  SAITransform
  Kinematics::globalFrame( taoDNode* node, const SAIVector & relativeDisplacement ) {
	
    SAIVector node_trans(3); 

    // Get rotation
    deQuaternion taoquat = node->frameGlobal()->rotation();
    deVector3 taoaxis;
    deFloat taoangle;
    taoquat.get(taoaxis,taoangle);
    SAIVector saiaxis(3);
    for (int ii(0); ii<3; ++ii) saiaxis[ii] = taoaxis[ii];
    SAIQuaternion saiquat(saiaxis,(double) taoangle);
 
    // Get translation
    for( int ii(0); ii < 3; ++ii )
      node_trans.elementAt(ii) = node->frameGlobal()->translation().elementAt(ii);

    if( 0 != relativeDisplacement.size() )
      node_trans += saiquat * relativeDisplacement;

    SAITransform ans(saiquat,node_trans);

    return ans;
  }


  SAIMatrix
  Kinematics::JacobianAtPoint( taoDNode* node, const SAIVector & globalPointOnLink ) {

    SAIMatrix J(6,noj_);
    SAIVector node_trans(3); 
    SAIMatrix id3(3,3); 
    id3.identity();

    // find displacement from desired node to global frame
    if( 0 == globalPointOnLink.size() )
      for( int ii(0); ii < 3; ++ii )
	node_trans.elementAt(ii) = 
	  node->frameGlobal()->translation().elementAt(ii);// this is the frame at the joint
    else
      node_trans = globalPointOnLink;

    // cross product operator
    SAIMatrix phat(3,3);
    phat.zero();
    phat.elementAt(0,1) = - node_trans.elementAt(2);
    phat.elementAt(0,2) = node_trans.elementAt(1);
    phat.elementAt(1,0) = node_trans.elementAt(2);
    phat.elementAt(1,2) = -node_trans.elementAt(0);
    phat.elementAt(2,0) = -node_trans.elementAt(1);
    phat.elementAt(2,1) = node_trans.elementAt(0);

    // create a position displacement matrix - 
    // similar to spatial operator but with rotations equal to identity
    SAIMatrix disp_matrix(6,6);
    for( int ii(0); ii < 3; ++ii )
      for( int jj(0); jj < 3; ++jj ) {
	disp_matrix.elementAt(ii,jj) = id3.elementAt(ii,jj);
	disp_matrix.elementAt(ii+3,jj+3) = id3.elementAt(ii,jj);
      }
    for( int ii(0); ii < 3; ++ii )
      for( int jj(0); jj < 3; ++jj ) {
	disp_matrix.elementAt(ii,jj+3) = -phat.elementAt(ii,jj);
      }

    // compute Jacobian wrt the global frame with zero displacement. 
    while( node->getDParent() != NULL ) {

      taoJoint* joint = node->getJointList();
      taoJointType jointType = joint->getType();

      if (( jointType == TAO_JOINT_REVOLUTE ) || ( jointType == TAO_JOINT_PRISMATIC )) {

	deVector6 deVector = (static_cast<taoJointDOF1*>(joint))->getJg();
      
	int id = node->getID();

	for( int i(0); i < 6; ++i )
	  J.elementAt(i,id) = deVector.elementAt(i);
      
      }
      else {
	cerr << "WARNING\n"
	     << "WARNING: Kinematics::JacobianAtPoint(): jointType " << jointType
	     << "not handled\n";
      }
      node = node->getDParent();
    }

    // add displacement of node (at the joint) or desired global coordinate
    J = disp_matrix * J;

    return J;
  }


  void Kinematics::robotControlModel( RobotControlModel* robmodel ) {

    robmodel_ = robmodel;

    // Getting references
    branchingModel_ = robmodel_->branching();
    noj_ = branchingModel_->numJoints();
    left_foot_ = branchingModel_->findLink("Left_Foot");
    right_foot_ = branchingModel_->findLink("Right_Foot");
    
    Jcom_.setSize(3,noj_);
  }


  void Kinematics::onUpdate( SAIVector const & jointAngles,
			     SAIVector const & jointVelocities ) {

    curQ_ = jointAngles;
    curDQ_ = jointVelocities;

    for( int i = 0; i < noj_; i++ ) {
      taoDNode* node = branchingModel_->idToNodeMap()[i];
      //deFloat curQ; curQ = (deFloat) dataContainer->jointAngles[i];
      //deFloat curDQ; curDQ = (deFloat) dataContainer->jointVelocities[i];
      node->getJointList()[0].setQ( &curQ_[i] );	
      node->getJointList()[0].setDQ( &curDQ_[i] );
      //node->getJointList()[0].setQ( &curQ );
      //node->getJointList()[0].setDQ( &curDQ );
      node->getJointList()[0].zeroTau();
    }

    taoDynamics::updateTransformation(branchingModel_->rootNode());
    taoDynamics::globalJacobian(branchingModel_->rootNode());

    calculateCOM();
    calculateJacobianCOM();

    calculateFrameBetweenFeetJoints();
  }


  bool Kinematics::
  checkDisplayJointLimits(std::ostream * os, char const * title, char const * prefix)
  {
    SAIVector const & upper(branchingModel_->upperJointLimits());
    SAIVector const & lower(branchingModel_->lowerJointLimits());
    bool ok(true);
    for (int ii(0); ii < noj_; ++ii) {
      if ((curQ_[ii] > upper[ii]) || (curQ_[ii] < lower[ii])) {
	if (os) {
	  if (ok && title)	// first joint to fail, print title
	    *os << title << "\n";
	  if (prefix)
	    *os << prefix;
	  if (curQ_[ii] > upper[ii])
	    *os << "curQ[" << ii << "] too large: cur = " << curQ_[ii]
		<< " > " << upper[ii] << " = upper\n";
	  else
	    *os << "curQ[" << ii << "] too small: cur = " << curQ_[ii]
		<< " < " << lower[ii] << " = lower\n";
	}
	ok = false;
      }
    }
    return ok;
  }

}
