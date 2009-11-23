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
   \file MobileManipulatorTaskModel.cpp
   \author Roland Philippsen and Luis Sentis
*/

#include "MobileManipulatorTaskModel.hpp"
#include <wbcnet/log.hpp>
#include <saimatrix/SAILapack.h>
#include <wbc/core/Dynamics.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/BehaviorDescription.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace {
  
  class MMView : public wbc::TaskModelView {
  public:
    explicit MMView(wbc::MobileManipulatorTaskModel const * _mm): mm(_mm) {}
    
    virtual SAIMatrix const * massInertia() const { return mm->massInertia; }

    virtual SAIMatrix const * invMassInertia() const { return mm->invMassInertia; }

    virtual SAIMatrix const * gravityForce() const { return mm->gravityForce; }

    virtual SAIMatrix const * coriolisCentrifugalForce() const { return mm->coriolisCentrifugalForce; }

    virtual SAIMatrix const * LambdaStar(int taskID) const {
      if (mm->LambdaStar->size() > taskID)
	return (*mm->LambdaStar)[taskID];
      return 0;
    }

    virtual SAIMatrix const * JStar(int taskID) const {
      if (mm->JStar->size() > taskID)
	return (*mm->JStar)[taskID];
      return 0;
    }
    
    wbc::MobileManipulatorTaskModel const * mm;
  };
  
}


namespace wbc {
  

  MobileManipulatorTaskModel::
  MobileManipulatorTaskModel(int _ndof, wbcnet::endian_mode_t endian_mode)
    : TaskModelBase(endian_mode),
      ndof(_ndof),
      m_view(0)
  {
    massInertia = AddIndepMx("massInertia");
    invMassInertia = AddIndepMx("invMassInertia");
    gravityForce = AddIndepMx("gravityForce");
    coriolisCentrifugalForce = AddIndepMx("coriolisCentrifugalForce");
    LambdaStar = AddTaskMx("LambdaStar", 0);
    JStar = AddTaskMx("JStar", 0);
    muStar = AddTaskMx("muStar", 0);
    pStar = AddTaskMx("pStar", 0);
    SingularValues = AddTaskMx("SingularValues", 0);
  }
  

  MobileManipulatorTaskModel::
  ~MobileManipulatorTaskModel()
  {
    delete m_view;
  }
  
  
  TaskModelView const * MobileManipulatorTaskModel::
  GetView() const
  {
    if ( ! m_view)
      m_view = new MMView(this);
    return m_view;
  }
  
  
  bool MobileManipulatorTaskModel::
  Update(BehaviorDescription const & behavior,
	 RobotControlModel const & robmodel)
  {
    Dynamics * dynamics(robmodel.dynamics());
    
    *massInertia = dynamics->massInertia();
    *invMassInertia = dynamics->invMassInertia();
    gravityForce->setSize(dynamics->gravityForce().size(), 1, false);
    gravityForce->setColumn(0, dynamics->gravityForce());
    coriolisCentrifugalForce->setSize(dynamics->coriolisCentrifugalForce().size(), 1, false);
    coriolisCentrifugalForce->setColumn(0, dynamics->coriolisCentrifugalForce());
    
    BehaviorDescription::task_set_vector task_sets(behavior.allTaskSets());
    size_t ttitask(0);
    
    for (BehaviorDescription::task_set_vector::const_iterator iset(task_sets.begin());
        iset != task_sets.end(); ++iset) {

      SAIMatrix NStar(ndof, ndof);
      NStar.identity();

      for (TaskSet::TaskList2D::const_iterator itask((*iset)->begin());
          itask != (*iset)->end(); ++itask) {

        SAIMatrix Jlevel(0, 0);
        appendJacobians(Jlevel, itask);

        if (Jlevel.fEmpty()) {
          LOG_DEBUG (logger, "wbc::MobileManipulatorTaskModel::Update(): empty Jacobian for taskID " << ttitask);
          (*JStar)[ttitask]->SetSize(0, 0);
          (*LambdaStar)[ttitask]->SetSize(0, 0);
          (*muStar)[ttitask]->SetSize(0, 0);
          (*pStar)[ttitask]->SetSize(0, 0);
          (*SingularValues)[ttitask]->SetSize(0, 0);
        }
        else {
          SAIMatrix & JStar_(*(*JStar)[ttitask]);
          SAIMatrix & LambdaStar_(*(*LambdaStar)[ttitask]);
          JStar_ = Jlevel * NStar;
          SAIMatrix const JStarT(JStar_.transpose());
          int const status(sai_lapack_stable_svd_inverse(JStar_ * (*invMassInertia) * JStarT,
                1e-3,
                LambdaStar_));
          if (0 != status) {
            LOG_ERROR (logger,
                "wbc::MobileManipulatorTaskModel::Update(): sai_lapack_stable_svd_inverse() returned "
                << status);
            return false;
          }
          NStar = (SAIMatrix::identity(ndof) - (*invMassInertia) * JStarT * LambdaStar_ * JStar_) * NStar;

          *(*muStar)[ttitask] = LambdaStar_ * JStar_ * (*invMassInertia) * (*coriolisCentrifugalForce);
          *(*pStar)[ttitask] = LambdaStar_ * JStar_ * (*invMassInertia) * (*gravityForce);
        }

        ++ttitask;
      }
    }
    
    return true;
  }
  
  
  MobileManipulatorTaskModel * MobileManipulatorTaskModelFactory::
  Create(BranchingRepresentation * branching,
	 wbcnet::endian_mode_t endian_mode)
  {
    if (branching->numJoints() != branching->numActuatedJoints())
      LOG_WARN (logger,
		"wbc::MobileManipulatorTaskModelFactory: number of actuated joints " << branching->numActuatedJoints()
		<< " differs from total number of joints " << branching->numJoints());
    MobileManipulatorTaskModel * model(new MobileManipulatorTaskModel(branching->numActuatedJoints(), endian_mode));
    return model;
  }
  
}
