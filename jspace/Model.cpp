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
#include <wbc/util/tao_util.hpp>
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>


namespace jspace {
  
  
  Model::
  Model(wbc::tao_tree_info_s * kg_tree,
	wbc::tao_tree_info_s * cc_tree)
    : kg_tree_(kg_tree), cc_tree_(cc_tree)
  {
  }


  Model::
  ~Model()
  {
    delete kg_tree_;
    delete cc_tree_;
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
    for (size_t ii(0); ii < kg_tree_->info.size(); ++ii) {
      taoJoint * joint(kg_tree_->info[ii].node->getJointList());
      joint->setQ(&state.joint_angles_[ii]);
      joint->zeroDQ();
      joint->zeroDDQ();
      joint->zeroTau();
    }
    for (size_t ii(0); ii < cc_tree_->info.size(); ++ii) {
      taoJoint * joint(cc_tree_->info[ii].node->getJointList());
      joint->setQ(&state.joint_angles_[ii]);
      joint->setDQ(&state.joint_velocities_[ii]);
      joint->zeroDDQ();
      joint->zeroTau();
    }
  }
  
  
  size_t Model::
  getNNodes() const
  {
    return kg_tree_->info.size();
  }
  
  
  size_t Model::
  getNJoints() const
  {
    // one day this will be different...
    return getNNodes();
  }
  
  
  size_t Model::
  getNDOF() const
  {
    // one day this will be different...
    return getNNodes();
  }
  
  
  std::string Model::
  getNodeName(size_t id) const
  {
    std::string name("");
    if (kg_tree_->info.size() > id) {
      name = kg_tree_->info[id].link_name;
    }
    return name;
  }
  
  
  std::string Model::
  getJointName(size_t id) const
  {
    std::string name("");
    if (kg_tree_->info.size() > id) {
      name = kg_tree_->info[id].joint_name;
    }
    return name;
  }
  
  
  taoDNode * Model::
  getNode(size_t id) const
  {
    if (kg_tree_->info.size() > id) {
      return kg_tree_->info[id].node;
    }
    return 0;
  }
  
  
  taoDNode * Model::
  getNodeByName(std::string const & name_or_alias) const
  {
    for (size_t ii(0); ii < kg_tree_->info.size(); ++ii) {
      if (name_or_alias == kg_tree_->info[ii].link_name) {
	return  kg_tree_->info[ii].node;
      }
    }
    return 0;
  }
  
  
  taoDNode * Model::
  getNodeByJointName(std::string const & name_or_alias) const
  {
    for (size_t ii(0); ii < kg_tree_->info.size(); ++ii) {
      if (name_or_alias == kg_tree_->info[ii].joint_name) {
	return  kg_tree_->info[ii].node;
      }
    }
    return 0;
  }
  
  
  void Model::
  getJointLimits(std::vector<double> & joint_limits_lower,
		 std::vector<double> & joint_limits_upper) const
  {
    joint_limits_lower.resize(kg_tree_->info.size());
    joint_limits_upper.resize(kg_tree_->info.size());
    for (size_t ii(0); ii < kg_tree_->info.size(); ++ii) {
      joint_limits_lower[ii] = kg_tree_->info[ii].limit_lower;
      joint_limits_upper[ii] = kg_tree_->info[ii].limit_upper;
    }
  }
  
  
  void Model::
  updateKinematics()
  {
    taoDynamics::updateTransformation(kg_tree_->root);
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
    cerr << "IMPLEMENT jspace::Model::computeJacobian()!!!\n";
    return false;
  }
  
  
  bool Model::
  computeJacobian(taoDNode const * node,
		  SAIVector const & global_point,
		  SAIMatrix & jacobian) const
  {
    if ( ! node) {
      return false;
    }
    cerr << "IMPLEMENT jspace::Model::computeJacobian() with global point!!!\n";
    return false;
  }
  
  
  void Model::
  updateDynamics()
  {
    computeGravity();
    computeCoriolisCentrifugal();
  }
  
  
  void Model::
  computeGravity()
  {
    static deVector3 const earth_gravity(0, 0, -9.81);
    g_torque_.resize(kg_tree_->info.size());
    taoDynamics::invDynamics(kg_tree_->root, &earth_gravity);
    for (size_t ii(0); ii < kg_tree_->info.size(); ++ii) {
      taoJoint * joint(kg_tree_->info[ii].node->getJointList());
      joint->getTau(&g_torque_[ii]);
    }
  }
  
  
  bool Model::
  disableGravityCompensation(size_t index, bool disable)
  {
    if (getNDOF() <= index) {
      return true;
    }
    
    dof_set_t::const_iterator const idof(gravity_disabled_.find(index));
    
    if (idof == gravity_disabled_.end()) {
      // currently not disabled
      if (disable) {
	gravity_disabled_.insert(index);
      }
      return false;
    }
    
    // currently disabled
    if ( ! disable) {
      gravity_disabled_.erase(idof);
    }
    return true;
  }
  
  
  void Model::
  getGravity(SAIVector & gravity) const
  {
    gravity.setSize(g_torque_.size(), true);
    for (size_t ii(0); ii < g_torque_.size(); ++ii) {
      // Only copy over the gravity torque in case it has NOT been
      // DISabled for this index...
      if (gravity_disabled_.end() == gravity_disabled_.find(ii)) {
	gravity[ii] = g_torque_[ii];
      }
    }
  }
  
  
  void Model::
  computeCoriolisCentrifugal()
  {
    static deVector3 const zero_gravity(0, 0, 0);
    cc_torque_.resize(cc_tree_->info.size());
    taoDynamics::invDynamics(cc_tree_->root, &zero_gravity);
    for (size_t ii(0); ii < cc_tree_->info.size(); ++ii) {
      taoJoint * joint(cc_tree_->info[ii].node->getJointList());
      joint->getTau(&cc_torque_[ii]);
    }
  }
  
  
  void Model::
  getCoriolisCentrifugal(SAIVector & coriolis_centrifugal) const
  {
    coriolis_centrifugal.setSize(cc_torque_.size());
    for (size_t ii(0); ii < cc_torque_.size(); ++ii) {
      coriolis_centrifugal[ii] = cc_torque_[ii];
    }
  }
  
  
  void Model::
  computeMassInertia()
  {
    cerr << "IMPLEMENT jspace::Model::computeMassInertia()!!!\n";
  }
  
  
  void Model::
  getMassInertia(SAIMatrix & mass_inertia) const
  {
    // dummy: identity
    mass_inertia.setSize(kg_tree_->info.size(), kg_tree_->info.size(), true);
    for (size_t ii(0); ii < kg_tree_->info.size(); ++ii) {
      mass_inertia.elementAt(ii, ii) = 1;
    }
  }
  
  
  void Model::
  computeInverseMassInertia()
  {
    cerr << "IMPLEMENT jspace::Model::computeInverseMassInertia()!!!\n";
  }
  
  
  void Model::
  getInverseMassInertia(SAIMatrix & inverse_mass_inertia) const
  {
    // dummy: identity
    inverse_mass_inertia.setSize(kg_tree_->info.size(), kg_tree_->info.size(), true);
    for (size_t ii(0); ii < kg_tree_->info.size(); ++ii) {
      inverse_mass_inertia.elementAt(ii, ii) = 1;
    }
  }

}
