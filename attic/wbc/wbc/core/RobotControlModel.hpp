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
  \author     Luis Sentis
  \file       RobotControlModel.h
 */
//==============================================================================

#ifndef WBC_ROBOT_CONTROL_MODEL_H
#define WBC_ROBOT_CONTROL_MODEL_H

#include <string>
#ifndef WIN32
#include <sys/time.h>
#else
#include "wbcnet/win32/win32_compat.hpp"
#endif

namespace wbc {

  using namespace std;

  class BranchingRepresentation;
  class Kinematics;
  class Dynamics;
  class Contact;

  //======================================
  //  - CLASS DEFINITION -
  /*!
    \brief This is an implementation header
  */
  //======================================

  class RobotControlModel {
  public:

    /** Transfers ownership of branching_rep */
    explicit RobotControlModel(BranchingRepresentation * branching_rep);
		       
    virtual ~RobotControlModel();

    //! Get a reference to whole-body dynamic descriptors
    inline Dynamics* dynamics() const {return dynamics_;}

    //! Get a reference to whole-body kinematic descriptors
    inline Kinematics* kinematics() const {return kinematics_;}

    //! Get a reference to a whole-body contact description
    inline Contact* contact() const {return contact_;}

    //! Get a reference to a whole-body branching representation
    inline BranchingRepresentation* branching() const {return branchingModel_;}
  
    /** quick hack for tentative generality, as usual */
    inline void getForceDimension(size_t & nrows, size_t & ncolumns) const { nrows = 0; ncolumns = 0; }
  
    inline timeval time(){return curTime_;};

    inline void time(timeval time){curTime_=time;};

  private:
    //! references
    Dynamics* dynamics_;
    BranchingRepresentation* branchingModel_;
    Kinematics* kinematics_;
    Contact* contact_;
    timeval curTime_;
    //! Attributes
    string name_;
  };

}

#endif // WBC_ROBOT_CONTROL_MODEL_H
