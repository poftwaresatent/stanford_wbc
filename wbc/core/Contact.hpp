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
  \file				Contact.h
*/
//===========================================================================================

#ifndef WBC_CONTACT_H
#define WBC_CONTACT_H

#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <saimatrix/SAIVector.h>
#include <list>

namespace wbc {

  using namespace std;

  class Contact {

  public:

    Contact();

    typedef enum{ In_Contact, Free_Floating } ContactStatus;

    inline const ContactStatus contactStatus() const { return status_; }

    bool linkIsInContact( int nodeID ) const;
    bool linkContactForce( int nodeID, SAIVector & );
    
    // moments are given at sensor location
    bool linkContactMoment( int nodeID, SAIVector & );
    
    // note that contact forces and moments are opposite to reaction forces and moments
    inline const SAIVector & CoMContactForce() const { return CoMContactForce_; }
    inline const SAIVector & CoMContactMoment() const { return CoMContactMoment_; }
    
    // return center of pressure of contact link; if cop exists returns true
    bool cop( int linkID, SAIVector & cop );
    bool zmp( SAIVector & zmp );

    /** See RobotAPI::readSensors() about the format of the contacts matrix. */
    void onUpdate( SAIMatrix const & contacts );
    
    /** maps link IDs to centers of pressure */
    typedef map<int, SAIVector> cop_map_t;
    inline cop_map_t const & copMap() const { return copMap_; }
    
  protected:

    void robotControlModel( RobotControlModel* );

  private:

    //! Attributes.
    map< int, SAIVector > linkForceTable_;
    map< int, SAIVector > linkMomentTable_;
    ContactStatus status_;
    cop_map_t copMap_;
    SAIVector CoMContactForce_;
    SAIVector CoMContactMoment_;
    SAIVector zmp_;
  
    //! References
    RobotControlModel* robmodel_;	
    int noj_;
    taoDNode * right_foot_;
    taoDNode * left_foot_;
    taoDNode * right_hand_;
    taoDNode * left_hand_;
    int right_footID_;
    int left_footID_;
    int right_handID_;
    int left_handID_;
    
    /** Its implementation is based on the following formula:
     *
     * copX(i) ~= sensX(i) - f_rX(i) (sensV(i) - copV(i))/f_rV(i) - m_rY(i)/f_rV(i) 
     * copY(i) ~= sensY(i) - f_rY(i) (sensV(i) - copV(i))/f_rV(i) + m_rX(i)/f_rV(i) 
     *
     * Author: Luis Sentis
     *
     * Ref1: Modeling and control of multi-contact centers of pressure and internal forces in
     * humanoid robots, Luis Sentis, Jaeheung Park, Oussama Khatib, IROS 2009
     *
     * Ref2: Based on author's interpretation of Vukobratovic and Borovac paper, "Zero
     * Moment Point -- Thirty Five Years of its Life", International Journal of Humanoid Robots,
     * 1(1):157-173, 2004.
     */
    SAIVector calculate_cop( taoDNode * link, double surface_global_height );
    
    /** Its implementation is based on the following formula:
     *
     * zmpX(i) ~= comX(i) - f_rX(i) (comZ(i) - zmpZ(i))/f_rZ(i) - m_rY(i)/f_rZ(i) 
     * zmpY(i) ~= comY(i) - f_rY(i) (comZ(i) - zmpZ(i))/f_rZ(i) + m_rX(i)/f_rZ(i) 
     *
     * Author: Luis Sentis
     * Ref: Based on author's interpretiation of Vukobratovic and Borovac paper, "Zero
     * Moment Point -- Thirty Five Years of its Life", International Journal of Humanoid Robots,
     * 1(1):157-173, 2004.
     */
    SAIVector calculate_zmp( double floor_height );
    
    void calculateCoMFMContribution( taoDNode * link );

    //! Friends
    friend class RobotControlModel;
  };

}

#endif // WBC_CONTACT_H
