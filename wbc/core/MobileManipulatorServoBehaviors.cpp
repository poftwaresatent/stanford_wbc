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
   \file MobileManipulatorServoBehaviors.cpp
   \author Roland Philippsen and Luis Sentis
*/

#include "MobileManipulatorServoBehaviors.hpp"
#include "BehaviorDescription.hpp"
#include "../core/MobileManipulatorTaskModel.hpp"
#include "../core/BranchingRepresentation.hpp"
#include <wbcnet/log.hpp>
#include <sstream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {

  
  bool MobileManipulatorServoBehaviors::
  updateTorques(BehaviorDescription * behavior,
		RobotControlModel * robmodel,
		wbcnet::TaskModelAPI const * taskModel,
		SAIVector & generalizedTorques)
  {
    MobileManipulatorTaskModel const * mmtm(dynamic_cast<MobileManipulatorTaskModel const *>(taskModel));
    if ( ! mmtm) {
      LOG_ERROR (logger,
		 "wbc::MobileManipulatorServoBehaviors::updateTorques(): invalid TaskModelAPI subtype");
      return false;
    }

    TaskSet * activeTaskSet(behavior->activeTaskSet());
    int const taskset_index(behavior->lookupTaskSetIndex(activeTaskSet));
    if (0 > taskset_index)  {
      LOG_ERROR (logger,
          "wbc::MobileManipulatorServoBehaviors::updateTorques(): invalid task set index " << taskset_index);
      return false;
    }

    int const ndof(robmodel->branching()->numActuatedJoints());
    generalizedTorques.setSize(ndof, true);

    SAIVector prevtorque(ndof);
    SAIMatrixAPI const & AInv(*mmtm->invMassInertia);
    int level_index(0);

    for (TaskSet::TaskList2D::const_iterator its(activeTaskSet->begin());
        its != activeTaskSet->end();
        ++its,
        ++level_index)
    {
      int const task_id(mmtm->ComputeTaskID(taskset_index, level_index));
      if (0 > task_id) {
        LOG_ERROR (logger,
            "wbc::MobileManipulatorServoBehaviors::updateTorques(): invalid task ID " << task_id
            << " for task set " << taskset_index << " / task level " << level_index);
        return false;
      }

      SAIMatrix const & LambdaStar(*(*mmtm->LambdaStar)[task_id]);
      SAIMatrix const & JStar(*(*mmtm->JStar)[task_id]);
      SAIVector const muStar((*mmtm->muStar)[task_id]->subvector(0,0,LambdaStar.row()));
      SAIVector const pStar((*mmtm->pStar)[task_id]->subvector(0,0,LambdaStar.row()));
      SAIMatrix const & SingularValues(*(*mmtm->SingularValues)[task_id]);
      SAIVector const FComp(LambdaStar * JStar * AInv * prevtorque);
      SAIVector accel_des;
      TaskSet::appendServos(its, accel_des);

      generalizedTorques += JStar.transpose() * (LambdaStar * accel_des + muStar + pStar - FComp);

      if (logger->isDebugEnabled()) {
      std::ostringstream msg;
      msg << "wbc::MobileManipulatorServoBehaviors::updateTorques():\n"
          << "  taskset_index: " << taskset_index << "\n"
          << "  level_index:   " << level_index << "\n"
          << "  task ID:       " << task_id << "\n";
      prevtorque.prettyPrint(msg, "  previous torque", "    ");
      generalizedTorques.prettyPrint(msg, "  generalized torques", "    ");
      JStar.prettyPrint(msg, "  JStar", "    ");
      LambdaStar.prettyPrint(msg, "  LambdaStar", "    ");
      accel_des.prettyPrint(msg, "  desired acceleration", "    ");
      FComp.prettyPrint(msg, "  FComp", "    ");
      muStar.prettyPrint(msg, "  muStar", "    ");
      pStar.prettyPrint(msg, "  pStar", "    ");
      LOG_DEBUG (logger, msg.str());
      }

      prevtorque = generalizedTorques;
    }
    
    return true;
  }
  
  
  void MobileManipulatorServoBehaviors::
  record( Time const & now )
  {
  }
  
}
