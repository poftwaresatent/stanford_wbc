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
   \file MobileManipulatorTaskModel.hpp
   \author Roland Philippsen and Luis Sentis
*/

#ifndef WBC_MOBILE_MANIPULATOR_TASK_MODEL_HPP
#define WBC_MOBILE_MANIPULATOR_TASK_MODEL_HPP

#include <wbc/core/TaskModelBase.hpp>
#include <wbc/core/TaskModelFactory.hpp>

namespace wbc {
  
  class MobileManipulatorTaskModel
    : public TaskModelBase
  {
  public:
    MobileManipulatorTaskModel(int ndof, wbcnet::endian_mode_t endian_mode);
    virtual ~MobileManipulatorTaskModel();
    
    int const ndof;
    
    SAIMatrixAPI * massInertia;
    SAIMatrixAPI * invMassInertia;
    SAIMatrixAPI * gravityForce;
    SAIMatrixAPI * coriolisCentrifugalForce;
    
    matrix_array_t * LambdaStar;
    matrix_array_t * JStar;
    matrix_array_t * muStar;
    matrix_array_t * pStar;
    matrix_array_t * SingularValues;

    virtual TaskModelView const * GetView() const;
    
    virtual bool Update(BehaviorDescription const & behavior,
			RobotControlModel const & robmodel);
    
  protected:
    mutable TaskModelView * m_view;
  };
  
  
  class MobileManipulatorTaskModelFactory
    : public TaskModelFactoryAPI
  {
  public:
    virtual MobileManipulatorTaskModel * Create(BranchingRepresentation * branching,
						wbcnet::endian_mode_t endian_mode);
  };
  
}

#endif // WBC_MOBILE_MANIPULATOR_TASK_MODEL_HPP
