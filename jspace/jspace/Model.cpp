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
#include "tao_util.hpp"
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>

#undef DEBUG


static deVector3 const zero_gravity(0, 0, 0);
static deVector3 const earth_gravity(0, 0, -9.81);


// Beware: no bound checks!
size_t squareToTriangularIndex(size_t irow, size_t icol, size_t dim)
{
  if (0 == irow) {
    return icol;
  }
  if (0 == icol) {
    return irow;
  }
  if (irow > icol) {
    // should have a lookup table for icol * (icol + 1) / 2
    return irow + dim * icol - icol * (icol + 1) / 2;
  }
  return icol + dim * irow - irow * (irow + 1) / 2;
}


namespace jspace {
  
  
  Model::
  Model()
    : ndof_(0),
      kgm_tree_(0),
      cc_tree_(0)
  {
  }
  
  
  int Model::
  init(tao_tree_info_s * kgm_tree,
       tao_tree_info_s * cc_tree,
       std::ostream * msg)
  {
    int const status(tao_consistency_check(kgm_tree->root, msg));
    if (0 != status) {
      return status;
    }
    
    if (kgm_tree_) {
      if (msg) {
	*msg << "jspace::Model::init(): already initialized\n";
      }
      return -1;
    }
    
    if ( ! kgm_tree->sort()) {
      if (msg) {
	*msg << "jspace::Model::init(): could not sort KGM nodes according to IDs\n";
      }
      return -2;
    }
    
    if ((0 != cc_tree) && ( ! cc_tree->sort())) {
      if (msg) {
	*msg << "jspace::Model::init(): could not sort CC nodes according to IDs\n";
      }
      return -3;
    }
    
    // Create ancestry table of all nodes in the KGM tree, for correct
    // (and slightly more efficient) computation of the Jacobian.
    ancestry_table_.clear();	// just paranoid...
    typedef tao_tree_info_s::node_info_t::const_iterator cit_t;
    cit_t in(kgm_tree->info.begin());
    cit_t iend(kgm_tree->info.end());
    for (/**/; in != iend; ++in) {
      // ...paranoid checks...
      if ( ! in->node) {
	if (msg) {
	  *msg << "jspace::Model::init(): NULL node at entry #" << in->id << "\n";
	}
	return -4;
      }
      if (in->id != in->node->getID()) {
	if (msg) {
	  *msg << "jspace::Model::init(): node ID of entry #" << in->id << " is " << in->node->getID() << "\n";
	}
	return -5;
      }
      ancestry_list_t & alist(ancestry_table_[in->node]); // first reference creates the instance
      // walk up the ancestry, append each parent to the list of
      // ancestors of this node
      for (taoDNode * node(in->node); 0 != node; node = node->getDParent()) {
	ancestry_entry_s entry;
	entry.id = node->getID();
	entry.joint = node->getJointList();
	if (0 != entry.joint) {
	  alist.push_back(entry);
	}
      }
    }
    
    kgm_tree_ = kgm_tree;
    cc_tree_ = cc_tree;
    ndof_ = kgm_tree->info.size();
    
    return 0;
  }
  
  
  Model::
  ~Model()
  {
    delete kgm_tree_;
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
    for (size_t ii(0); ii < ndof_; ++ii) {
      taoJoint * joint(kgm_tree_->info[ii].joint);
      joint->setQ(&const_cast<State&>(state).position_.coeffRef(ii));
      joint->zeroDQ();
      joint->zeroDDQ();
      joint->zeroTau();
    }
    if (cc_tree_) {
      for (size_t ii(0); ii < ndof_; ++ii) {
	taoJoint * joint(cc_tree_->info[ii].joint);
	joint->setQ(&const_cast<State&>(state).position_.coeffRef(ii));
	joint->setDQ(&const_cast<State&>(state).velocity_.coeffRef(ii));
	joint->zeroDDQ();
	joint->zeroTau();
      }
    }
  }
  
  
  size_t Model::
  getNNodes() const
  {
    return ndof_;
  }
  
  
  size_t Model::
  getNJoints() const
  {
    // one day this will be different...
    return ndof_;
  }
  
  
  size_t Model::
  getNDOF() const
  {
    // one day this will be different...
    return ndof_;
  }
  
  
  std::string Model::
  getNodeName(size_t id) const
  {
    std::string name("");
    if (ndof_ > id) {
      name = kgm_tree_->info[id].link_name;
    }
    return name;
  }
  
  
  std::string Model::
  getJointName(size_t id) const
  {
    std::string name("");
    if (ndof_ > id) {
      name = kgm_tree_->info[id].joint_name;
    }
    return name;
  }
  
  
  taoDNode * Model::
  getNode(size_t id) const
  {
    if (ndof_ > id) {
      return kgm_tree_->info[id].node;
    }
    return 0;
  }
  
  
  taoDNode * Model::
  getNodeByName(std::string const & name) const
  {
    for (size_t ii(0); ii < ndof_; ++ii) {
      if (name == kgm_tree_->info[ii].link_name) {
	return  kgm_tree_->info[ii].node;
      }
    }
    return 0;
  }
  
  
  taoDNode * Model::
  getNodeByJointName(std::string const & name) const
  {
    for (size_t ii(0); ii < ndof_; ++ii) {
      if (name == kgm_tree_->info[ii].joint_name) {
	return  kgm_tree_->info[ii].node;
      }
    }
    return 0;
  }
  
  
  void Model::
  getJointLimits(Vector & joint_limits_lower,
		 Vector & joint_limits_upper) const
  {
    joint_limits_lower.resize(ndof_);
    joint_limits_upper.resize(ndof_);
    for (size_t ii(0); ii < ndof_; ++ii) {
      joint_limits_lower[ii] = kgm_tree_->info[ii].limit_lower;
      joint_limits_upper[ii] = kgm_tree_->info[ii].limit_upper;
    }
  }
  
  
  void Model::
  updateKinematics()
  {
    taoDynamics::updateTransformation(kgm_tree_->root);
    taoDynamics::globalJacobian(kgm_tree_->root);
    if (cc_tree_) {
      taoDynamics::updateTransformation(cc_tree_->root);
      taoDynamics::globalJacobian(cc_tree_->root);
    }
  }
  
  
  bool Model::
  getGlobalFrame(taoDNode const * node,
		 Transform & global_transform) const
  {
    if ( ! node) {
      return false;
    }
    
    deFrame const * tao_frame(node->frameGlobal());
    deQuaternion const & tao_quat(tao_frame->rotation());
    deVector3 const & tao_trans(tao_frame->translation());
    
#warning "TO DO: maybe the other way around..."
    // beware: Eigen::Quaternion(w, x, y, z) puts w first, whereas
    // deQuaternion(qx, qy, qz, qw) puts w last. Of course.
    global_transform = Translation(tao_trans[0], tao_trans[1], tao_trans[2]);
    global_transform *= Quaternion(tao_quat[3], tao_quat[0], tao_quat[1], tao_quat[2]);
    
    return true;
  }
  
  
  bool Model::
  computeGlobalFrame(taoDNode const * node,
		     Transform const & local_transform,
		     Transform & global_transform) const
  {
    if ( ! getGlobalFrame(node, global_transform)) {
      return false;
    }
    global_transform = global_transform * local_transform;
    return true;
  }
  
  
  bool Model::
  computeGlobalFrame(taoDNode const * node,
		     Vector const & local_translation,
		     Transform & global_transform) const
  {
    if ( ! getGlobalFrame(node, global_transform)) {
      return false;
    }
    global_transform.translation() += global_transform.linear() * local_translation;
    return true;
  }
  
  
  bool Model::
  computeGlobalFrame(taoDNode const * node,
		     double local_x, double local_y, double local_z,
		     Transform & global_transform) const
  {
    if ( ! getGlobalFrame(node, global_transform)) {
      return false;
    }
    global_transform.translation() += global_transform.linear() * Eigen::Vector3d(local_x, local_y, local_z);
    return true;
  }

    
  bool Model::
  computeGlobalCOMFrame(taoDNode const * node,
			Transform & global_com_transform) const
  {
    if ( ! node) {
      return false;
    }
    
    deVector3 const * com(const_cast<taoDNode*>(node)->center());
    if ( ! com) {
      return getGlobalFrame(node, global_com_transform);
    }
    
    //// AAARGHHHL!!! Do NOT use "*com[0], *com[1], *com[2]" because
    //// deVector3 somehow gives garbage if you do that. Yet another
    //// case where hours were wasted because of TAO weirdness...
    return computeGlobalFrame(node, com->elementAt(0), com->elementAt(1), com->elementAt(2), global_com_transform);
  }
  
  
  bool Model::
  computeJacobian(taoDNode const * node,
		  Matrix & jacobian) const
  {
    if ( ! node) {
      return false;
    }
    deVector3 const & gpos(node->frameGlobal()->translation());
    return computeJacobian(node, gpos[0], gpos[1], gpos[2], jacobian);
  }
  
  
  bool Model::
  computeJacobian(taoDNode const * node,
		  double gx, double gy, double gz,
		  Matrix & jacobian) const
  {
    if ( ! node) {
      return false;
    }
    ancestry_table_t::const_iterator iae(ancestry_table_.find(const_cast<taoDNode*>(node)));
    if (iae == ancestry_table_.end()) {
      return false;
    }
    ancestry_list_t const & alist(iae->second);
    
#ifdef DEBUG
    fprintf(stderr, "computeJacobian()\ng: [% 4.2f % 4.2f % 4.2f]\n", gx, gy, gz);
#endif // DEBUG
    
    // \todo Implement support for more than one joint per node, and
    // 	more than one DOF per joint.
    jacobian = Matrix::Zero(6, ndof_);
    ancestry_list_t::const_iterator ia(alist.begin());
    ancestry_list_t::const_iterator iend(alist.end());
    for (/**/; ia != iend; ++ia) {
      deVector6 Jg_col;
      ia->joint->getJgColumns(&Jg_col);
      int const icol(ia->id);
      
#ifdef DEBUG
      fprintf(stderr, "iJg[%d]: [ % 4.2f % 4.2f % 4.2f % 4.2f % 4.2f % 4.2f]\n",
	      icol,
	      Jg_col.elementAt(0), Jg_col.elementAt(1), Jg_col.elementAt(2),
	      Jg_col.elementAt(3), Jg_col.elementAt(4), Jg_col.elementAt(5));
#endif // DEBUG
      
      for (size_t irow(0); irow < 6; ++irow) {
	jacobian.coeffRef(irow, icol) = Jg_col.elementAt(irow);
      }
      
      // Add the effect of the joint rotation on the translational
      // velocity at the global point (column-wise cross product with
      // [gx;gy;gz]). Note that Jg_col.elementAt(3) is the
      // contribution to omega_x etc, because the upper 3 elements of
      // Jg_col are v_x etc.  (And don't ask me why we have to
      // subtract the cross product, it probably got inverted
      // somewhere)
      jacobian.coeffRef(0, icol) -= -gz * Jg_col.elementAt(4) + gy * Jg_col.elementAt(5);
      jacobian.coeffRef(1, icol) -=  gz * Jg_col.elementAt(3) - gx * Jg_col.elementAt(5);
      jacobian.coeffRef(2, icol) -= -gy * Jg_col.elementAt(3) + gx * Jg_col.elementAt(4);
      
#ifdef DEBUG
      fprintf(stderr, "0Jg[%d]: [ % 4.2f % 4.2f % 4.2f % 4.2f % 4.2f % 4.2f]\n",
	      icol,
	      jacobian.coeff(0, icol), jacobian.coeff(1, icol), jacobian.coeff(2, icol),
	      jacobian.coeff(3, icol), jacobian.coeff(4, icol), jacobian.coeff(5, icol));
#endif // DEBUG
      
    }
    return true;
  }
  
  
  void Model::
  updateDynamics()
  {
    computeGravity();
    computeCoriolisCentrifugal();
    computeMassInertia();
    computeInverseMassInertia();
  }
  
  
  void Model::
  computeGravity()
  {
    g_torque_.resize(ndof_);
    taoDynamics::invDynamics(kgm_tree_->root, &earth_gravity);
    for (size_t ii(0); ii < ndof_; ++ii) {
      kgm_tree_->info[ii].joint->getTau(&g_torque_[ii]);
    }
  }
  
  
  bool Model::
  disableGravityCompensation(size_t index, bool disable)
  {
    if (ndof_ <= index) {
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
  
  
  bool Model::
  getGravity(Vector & gravity) const
  {
    if (0 == g_torque_.size()) {
      return false;
    }
    gravity = g_torque_;
    // knock away gravity torque from links that are already otherwise compensated
    dof_set_t::const_iterator iend(gravity_disabled_.end());
    for (dof_set_t::const_iterator ii(gravity_disabled_.begin()); ii != iend; ++ii) {
      gravity[*ii] = 0;
    }
    return true;
  }
  
  
  void Model::
  computeCoriolisCentrifugal()
  {
    if (cc_tree_) {
      cc_torque_.resize(ndof_);
      taoDynamics::invDynamics(cc_tree_->root, &zero_gravity);
      for (size_t ii(0); ii < ndof_; ++ii) {
	cc_tree_->info[ii].joint->getTau(&cc_torque_[ii]);
      }
    }
  }
  
  
  bool Model::
  getCoriolisCentrifugal(Vector & coriolis_centrifugal) const
  {
    if ( ! cc_tree_) {
      return false;
    }
    if (0 == cc_torque_.size()) {
      return false;
    }
    coriolis_centrifugal = cc_torque_;
    return true;
  }
  
  
  void Model::
  computeMassInertia()
  {
    if (a_upper_triangular_.empty()) {
      a_upper_triangular_.resize(ndof_ * (ndof_ + 1) / 2);
    }
    
    deFloat const one(1);
    for (size_t irow(0); irow < ndof_; ++irow) {
      taoJoint * joint(kgm_tree_->info[irow].joint);
      
      // Compute one column of A by solving inverse dynamics of the
      // corresponding joint having a unit acceleration, while all the
      // others remain fixed. This works on the kgm_tree because it
      // has zero speeds, thus the Coriolis-centrifgual effects are
      // zero, and by using zero gravity we get pure system dynamics:
      // force = mass * acceleration (in matrix form).
      joint->setDDQ(&one);
      taoDynamics::invDynamics(kgm_tree_->root, &zero_gravity);
      joint->zeroDDQ();
      
      // Retrieve the column of A by reading the joint torques
      // required for the column-selecting unit acceleration (into a
      // flattened upper triangular matrix).
      
      for (size_t icol(0); icol <= irow; ++icol) {
	kgm_tree_->info[icol].joint->getTau(&a_upper_triangular_[squareToTriangularIndex(irow, icol, ndof_)]);
      }
    }
    
    // Reset all the torques.
    for (size_t ii(0); ii < ndof_; ++ii) {
      kgm_tree_->info[ii].joint->zeroTau();
    }
  }
  
  
  bool Model::
  getMassInertia(Matrix & mass_inertia) const
  {
    if (a_upper_triangular_.empty()) {
      return false;
    }
    
    mass_inertia.resize(ndof_, ndof_);
    for (size_t irow(0); irow < ndof_; ++irow) {
      for (size_t icol(0); icol <= irow; ++icol) {
	mass_inertia.coeffRef(irow, icol) = a_upper_triangular_[squareToTriangularIndex(irow, icol, ndof_)];
	if (irow != icol) {
	  mass_inertia.coeffRef(icol, irow) = mass_inertia.coeff(irow, icol);
	}
      }
    }
    
    return true;
  }
  
  
  void Model::
  computeInverseMassInertia()
  {
    if (ainv_upper_triangular_.empty()) {
      ainv_upper_triangular_.resize(ndof_ * (ndof_ + 1) / 2);
    }
    
    deFloat const one(1);
    for (size_t irow(0); irow < ndof_; ++irow) {
      taoJoint * joint(kgm_tree_->info[irow].joint);
      
      // Compute one column of Ainv by solving forward dynamics of the
      // corresponding joint having a unit torque, while all the
      // others remain unactuated. This works on the kgm_tree because
      // it has zero speeds, thus the Coriolis-centrifgual effects are
      // zero, and by using zero gravity we get pure system dynamics:
      // acceleration = mass_inv * force (in matrix form).
      joint->setTau(&one);
      taoDynamics::fwdDynamics(kgm_tree_->root, &zero_gravity);
      joint->zeroTau();
      
      // Retrieve the column of Ainv by reading the joint
      // accelerations generated by the column-selecting unit torque
      // (into a flattened upper triangular matrix).
      for (size_t icol(0); icol <= irow; ++icol) {
	kgm_tree_->info[icol].joint->getDDQ(&ainv_upper_triangular_[squareToTriangularIndex(irow, icol, ndof_)]);
      }
    }
    
    // Reset all the accelerations.
    for (size_t ii(0); ii < ndof_; ++ii) {
      kgm_tree_->info[ii].joint->zeroDDQ();
    }
  }
  
  
  bool Model::
  getInverseMassInertia(Matrix & inverse_mass_inertia) const
  {
    if (ainv_upper_triangular_.empty()) {
      return false;
    }
    
    inverse_mass_inertia.resize(ndof_, ndof_);
    for (size_t irow(0); irow < ndof_; ++irow) {
      for (size_t icol(0); icol <= irow; ++icol) {
	inverse_mass_inertia.coeffRef(irow, icol) = ainv_upper_triangular_[squareToTriangularIndex(irow, icol, ndof_)];
	if (irow != icol) {
	  inverse_mass_inertia.coeffRef(icol, irow) = inverse_mass_inertia.coeff(irow, icol);
	}
      }
    }
    
    return true;
  }

}
