/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file jspace/Model.cpp
   \author Roland Philippsen, inspired by wbc/core code of Luis Sentis
*/

#include "Model.hpp"
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/core/Kinematics.hpp>
#include <wbc/core/Dynamics.hpp>
#include <tao/dynamics/taoNode.h>


namespace jspace {
  
  
  Model::
  Model(wbc::RobotControlModel * robmodel,
	bool cleanup_robmodel)
    : robmodel_(robmodel),
      cleanup_robmodel_(cleanup_robmodel)
  {
  }


  Model::
  ~Model()
  {
    if (cleanup_robmodel_) {
      delete robmodel_;
    }
  }
  
  
  void Model::
  update(State const & state)
  {
    setState(state);
    updateKinematics();
    updateDynamics();
  }
  
  
  void Model::
  setState(State const & state)
  {
    state_ = state;
  }
  
  
  int Model::
  getNNodes() const
  {
    return robmodel_->branching()->idToNodeMap().size();
  }
  
  
  int Model::
  getNJoints() const
  {
    return robmodel_->branching()->numJoints();
  }
  
  
  int Model::
  getNDOF() const
  {
    return robmodel_->branching()->numActuatedJoints();
  }
  
  
  taoDNode * Model::
  getNode(int id) const
  {
    return robmodel_->branching()->idToNodeMap()[id];
  }
  
  
  taoDNode * Model::
  getNodeByName(std::string const & name_or_alias) const
  {
    return robmodel_->branching()->findLink(name_or_alias);
  }
  
  
  taoDNode * Model::
  getNodeByJointName(std::string const & name_or_alias) const
  {
    return robmodel_->branching()->findJoint(name_or_alias);
  }
  
  
  void Model::
  updateKinematics()
  {
    robmodel_->kinematics()->onUpdate(state_.joint_angles_, state_.joint_velocities_);
  }
  
  
  bool Model::
  getGlobalFrame(taoDNode const * node,
		 SAITransform & global_transform) const
  {
    if ( ! node) {
      return false;
    }
    
    deFrame const * tao_frame(node->frameGlobal());
    deQuaternion const & tao_quat(tao_frame->rotation());
    deVector3 const & tao_trans(tao_frame->translation());
    
    global_transform.rotation().values(tao_quat[3], tao_quat[0], tao_quat[1], tao_quat[2]);
    global_transform.translation().values(tao_trans[0], tao_trans[1], tao_trans[2]);
    
    return true;
  }
  
  
  bool Model::
  computeGlobalFrame(taoDNode const * node,
		     SAITransform const & local_transform,
		     SAITransform & global_transform) const
  {
    SAITransform tmp;
    if ( ! getGlobalFrame(node, tmp)) {
      return false;
    }
    tmp.multiply(local_transform, global_transform);
    return true;
  }
  
  
  bool Model::
  computeJacobian(taoDNode const * node,
		  SAIMatrix & jacobian) const
  {
    if ( ! node) {
      return false;
    }
    
    static SAIVector const null(0);
    // wastes a tmp object...
    jacobian = robmodel_->kinematics()->JacobianAtPoint(node, null);
    
    return true;
  }
  
  
  bool Model::
  computeJacobian(taoDNode const * node,
		  SAIVector const & global_point,
		  SAIMatrix & jacobian) const
  {
    if ( ! node) {
      return false;
    }
    
    // wastes a tmp object...
    jacobian = robmodel_->kinematics()->JacobianAtPoint(node, global_point);
    
    return true;
  }
  
  
  void Model::
  updateDynamics()
  {
    robmodel_->dynamics()->onUpdate(state_.joint_angles_, state_.joint_velocities_);
  }
  
  
  void Model::
  computeGravity()
  {
    // for the time being, updateDynamics() does it for us
  }
  
  
  void Model::
  getGravity(SAIVector & gravity) const
  {
    gravity = robmodel_->dynamics()->gravityForce();
  }
  
  
  void Model::
  computeCoriolisCentrifugal()
  {
    // for the time being, updateDynamics() does it for us
  }
  
  
  void Model::
  getCoriolisCentrifugal(SAIVector & coriolis_centrifugal) const
  {
    coriolis_centrifugal = robmodel_->dynamics()->coriolisCentrifugalForce();
  }
  
  
  void Model::
  computeMassInertia()
  {
    // for the time being, updateDynamics() does it for us
  }
  
  
  void Model::
  getMassInertia(SAIMatrix & mass_inertia) const
  {
    mass_inertia = robmodel_->dynamics()->massInertia();
  }
  
  
  void Model::
  computeInverseMassInertia()
  {
    // for the time being, updateDynamics() does it for us
  }
  
  
  void Model::
  getInverseMassInertia(SAIMatrix & inverse_mass_inertia) const
  {
    inverse_mass_inertia = robmodel_->dynamics()->invMassInertia();
  }

}