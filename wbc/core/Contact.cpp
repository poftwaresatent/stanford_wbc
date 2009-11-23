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

//=====================================================================
/*!
  \author  Luis Sentis
  \file	   Contact.cpp
*/
//=====================================================================

#include <wbc/core/Contact.hpp>
#include <wbc/core/Kinematics.hpp>
#include <saimatrix/SAIVector3.h>
#include <tao/dynamics/taoDNode.h>
#include <math.h>

namespace {
  static SAIVector const nullVector;
}

namespace wbc {

  Contact::Contact()
    : status_(Free_Floating),
      right_foot_(0),
      left_foot_(0),
      right_hand_(0),
      left_hand_(0),
      right_footID_(-2),
      left_footID_(-2),
      right_handID_(-2),
      left_handID_(-2)
 {
    // Init contact variables
    CoMContactForce_.setSize(3,true);
    CoMContactMoment_.setSize(3,true);
  }


  void 
  Contact::onUpdate( SAIMatrix const & contacts ) {
    
    ////int nodeID;

    status_ = Free_Floating;
    
    // update force and moment tables
    linkForceTable_.clear();
    linkMomentTable_.clear();
    for (int icol(0); icol < contacts.column(); ++icol) {
      bool in_contact(false);
      status_ = Free_Floating;
      for (int irow(0); irow < 6; ++irow)
	if (fabs(contacts.elementAt(irow, icol)) > 1e-3) {
	  in_contact = true;
	  status_ = In_Contact;
	  break;
	}
      if (in_contact) {
	linkForceTable_.insert(make_pair(icol, contacts.subvector(0, icol, 3)));
	linkMomentTable_.insert(make_pair(icol, contacts.subvector(3, icol, 3)));
      }
    }
    
    // compute resultant forces/moments at CoM - this is needed to compute ZMP
    CoMContactForce_.zero();
    CoMContactMoment_.zero();

    // Humanoid developments frequently take advantage of hardcoded
    // references to feet and hands, which breaks on models that do
    // not have such links. We have to find a better way of keeping
    // them from interfering with non-free-floating robots such as
    // wheeled mobile manipulators.
#define LUIS_WORKING_ON_HUMANOID_STUFF
#ifdef LUIS_WORKING_ON_HUMANOID_STUFF
    if( linkIsInContact( right_footID_ ) )
      calculateCoMFMContribution( right_foot_ );

    if( linkIsInContact( left_footID_ ) )
      calculateCoMFMContribution( left_foot_ );
  
    // compute CoP's if needed
    double surface_global_height;
    double floor_height_set = false;
    double floor_height = 0.0;
    if( linkIsInContact( right_footID_ ) ) {
      // XXXX to do: cache surfaceDepth
      surface_global_height
	= robmodel_->kinematics()->globalFrame(right_foot_, nullVector).translation()[2]
	- robmodel_->branching()->surfaceDepth( right_foot_ );
    
      copMap_[right_footID_] = 
	calculate_cop( right_foot_, surface_global_height );

      floor_height = surface_global_height;
      floor_height_set = true;
    }
  
    if( linkIsInContact( left_footID_ ) ) {
      // XXXX to do: cache surfaceDepth
      surface_global_height
	= robmodel_->kinematics()->globalFrame(left_foot_, nullVector).translation()[2]
	- robmodel_->branching()->surfaceDepth( left_foot_ );
    
      copMap_[left_footID_] = 
	calculate_cop( left_foot_, surface_global_height );

      if( !floor_height_set || surface_global_height < floor_height ) {

	floor_height = surface_global_height;
	floor_height_set = true;
      }
    }

    // compute ZMP
    if( status_ == In_Contact ) {
    
      zmp_ = calculate_zmp( floor_height );    
    }
#endif // LUIS_WORKING_ON_HUMANOID_STUFF
  }

  void
  Contact::calculateCoMFMContribution( taoDNode * link ) {

    SAIVector Fr;
    SAIVector distance;
    int id = link->getID();
    Fr = linkForceTable_[id];
    Fr.append( linkMomentTable_[id] );
  
    // distance from contact to com
    SAIVector relative_sensor_pos = robmodel_->branching()->forceSensorWRTJointFrame( link );
    SAIVector global_sensor_pos
      = robmodel_->kinematics()->globalFrame( link, relative_sensor_pos ).translation();;
    distance = global_sensor_pos - robmodel_->kinematics()->COM();
  
    // spatial operator
    SAIMatrix distance_cross = distance.cross();
    SAIMatrix I33(3,3); I33.identity();
    SAIMatrix Z33(3,3); Z33.zero();
    SAIMatrix spatial_operator = I33;
    spatial_operator.appendHorizontally( Z33 );
    SAIMatrix foo = distance_cross;
    foo.appendHorizontally( I33 );
    spatial_operator.appendVertically( foo );
  
    SAIVector Fcom = spatial_operator * Fr;
    CoMContactForce_ += Fcom.subvector(0,3);
    CoMContactMoment_ += Fcom.subvector(3,3);
  }

  bool
  Contact::cop( int linkID, SAIVector & cop ) {

    if( linkIsInContact( linkID ) ) {
    
      cop = copMap_[linkID];
      return true;
    }
  
    return false;
  }


  bool 
  Contact::zmp( SAIVector & zmp ) {

    if( status_ == In_Contact ) {

      zmp = zmp_;
      return true;
    }

    return false;
  }


  SAIVector
  Contact::calculate_cop( taoDNode * link, double surface_global_height ) {

    SAIVector cop(3), sens(3), fr(3), mr(3);

    SAIVector contact_force_at_sensor;
    int const id(link->getID());
    linkContactForce( id, contact_force_at_sensor );
    SAIVector contact_moment_at_sensor;
    linkContactMoment( id, contact_moment_at_sensor );

    // Get sensor position of link
    SAIVector relative_sensor_pos = robmodel_->branching()->forceSensorWRTJointFrame( link );
    SAIVector global_sensor_pos
      = robmodel_->kinematics()->globalFrame( link, relative_sensor_pos ).translation();;

    sens[0] = global_sensor_pos[0];
    sens[1] = global_sensor_pos[1];
    sens[2] = global_sensor_pos[2];
    fr[0] = -contact_force_at_sensor[0];
    fr[1] = -contact_force_at_sensor[1];
    fr[2] = -contact_force_at_sensor[2];
    mr[0] = -contact_moment_at_sensor[0];
    mr[1] = -contact_moment_at_sensor[1];
    mr[2] = -contact_moment_at_sensor[2];

    // here the moment component between the sensor and com link location is ignore.
    
    cop[0] = sens[0] - fr[0]*( sens[2] - surface_global_height ) / fr[2] - mr[1] / fr[2];
    cop[1] = sens[1] - fr[1]*( sens[2] - surface_global_height ) / fr[2] + mr[0] / fr[2];
    cop[2] = surface_global_height;

    return cop;
  }


  SAIVector 
  Contact::calculate_zmp( double floor_height ) {

    SAIVector zmp(3), com(3), fr(3), mr(3);

    com = robmodel_->kinematics()->COM();

    fr[0] = -CoMContactForce_[0];
    fr[1] = -CoMContactForce_[1];
    fr[2] = -CoMContactForce_[2];
    mr[0] = -CoMContactMoment_[0];
    mr[1] = -CoMContactMoment_[1];
    mr[2] = -CoMContactMoment_[2];
  
    zmp[0] = com[0] - fr[0] * ( com[2] - floor_height ) / fr[2] - mr[1] / fr[2];
    zmp[1] = com[1] - fr[1] * ( com[2] - floor_height ) / fr[2] + mr[0] / fr[2];
    zmp[2] = floor_height;

    return zmp;
  }


  void Contact::robotControlModel( RobotControlModel* robModel ) {

    robmodel_ = robModel;

    // Getting references
    noj_ = robmodel_->branching()->numJoints();

    // initialize maps
    copMap_.clear();
    right_footID_ = -2;
    left_footID_ = -2;
    right_handID_ = -2;
    left_handID_ = -2;
    SAIVector const zero(3);
    BranchingRepresentation * branching(robModel->branching());
    right_foot_ = branching->findLink("Right_Foot");
    if (right_foot_) {
      right_footID_ = right_foot_->getID();
      copMap_.insert(make_pair(right_footID_, zero));
    }
    left_foot_ = branching->findLink("Left_Foot");
    if (left_foot_) {
      left_footID_ = left_foot_->getID();
      copMap_.insert(make_pair(left_footID_, zero));
    }
    if (right_hand_) {
      right_handID_ = right_hand_->getID();
      copMap_.insert(make_pair(right_handID_, zero));
    }
    left_hand_ = branching->findLink("Left_Hand");
    if (left_hand_) {
      left_handID_ = left_hand_->getID();
      copMap_.insert(make_pair(left_handID_, zero));
    }
  }

  bool Contact::linkIsInContact( int nodeID ) const {
    if (nodeID < 0) // rootNode or non-initialized, e.g. we have no "left foot"
      return false;
    return linkForceTable_.find( nodeID ) != linkForceTable_.end();
  }

  bool Contact::linkContactForce( int nodeID, SAIVector & force ) {

    if( linkForceTable_.find( nodeID ) != linkForceTable_.end() ) {

      force = linkForceTable_[nodeID];
      return true;
    }

    return false;
  }
  
  
  bool Contact::linkContactMoment( int nodeID, SAIVector & moment )
  {
    if( linkMomentTable_.find( nodeID ) != linkMomentTable_.end() ) {

      moment = linkMomentTable_[nodeID];
      return true;
    }

    return false;
  }
  
}
