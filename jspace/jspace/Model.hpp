/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
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
   \file jspace/Model.hpp
   \author Roland Philippsen, inspired by wbc/core code of Luis Sentis
*/

#ifndef JSPACE_MODEL_HPP
#define JSPACE_MODEL_HPP

#include <jspace/State.hpp>
#include <jspace/wrap_eigen.hpp>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>


namespace RigidBodyDynamics {
  class Model;
}


namespace jspace {
  
  
  /**
     \todo Now that we have RBDL, maybe we could make the
     jspace::Model stateless. After all, that was just a hack imposed
     by TAO's horrible way of storing state dispersed throughout its
     tree structure...
     
     \todo While TAO was computing Jacobians for points specified in
     global coordinates, RBDL uses body-local coordinates. For now,
     the jspace::Model interface keeps using the global frame, but
     probably a more sensible way would be to provide both explicitly
     with appropriately named methods.
  */
  class Model
  {
  public:
    /**
       Value returned by getNodeByName and friends when no matching
       node can be found.
    */
    static size_t const INVALID_NODE;
    
    /** Please use the init() method in order to initialize your
	jspace::Model. It does some sanity checking, and error
	handling from within a constructor is just not so great.
    */
    Model();
    
    ~Model();
    
    /** Initialize with an existing RigidBodyDynamics::Model instance.
	
	\note Transfers ownership of the given
	RigidBodyDynamics::Model. It will be deleted when this
	jspace::Model instance is destructed.

	\return 0 on success (always returns 0 since switching from
	TAO to RBDL).
    */
    int init(RigidBodyDynamics::Model * rbdl_model);
    
    //////////////////////////////////////////////////
    // fire-and-forget facet
    
    /** Calls setState(), updateKinematics(), and
	updateDynamics(). After calling the update() method, you can
	use any of the other methods without worrying whether you have
	already called the corresponding computeFoo() method.
	
	\note The given state has to have the correct dimensions, but
	this is not checked by the implementation. If the given state
	has too few dimensions, then some positions and velocities of
	the model will remain at their old values. If there are too
	many dimensions, they will be ignored.
    */
    void update(State const & state);
    
    /** Inform the model about the joint state. We used to have to
	separate the state update from the computation of the various
	quantities in order to efficiently use the TAO tree
	representation, which forced us to distribute the state over
	its nodes before computing the model. With RBDL, that is not
	necessary anymore, but kept in the API for now to avoid
	breaking too much existing code.
	
	\note The given state has to have the correct dimensions, but
	this is not checked by the implementation. If the given state
	has too few dimensions, then some positions and velocities of
	the model will remain at their old values. If there are too
	many dimensions, they will be ignored.
	
	\todo We do not use TAO anymore, should be able to make
	jspace::Model stateless now.
    */
    void setState(State const & state);
    
    /** Retrieve the state passed to setState() (or update(), for that
	matter). */
    inline State const & getState() const { return state_; }
    
    //////////////////////////////////////////////////
    // Bare tree accessors.
    
    /** Compute or retrieve the cached number of nodes in the
	robot.
	
	\note
	- the root node is NOT included in this count.
	- in principle, each node can have any number of joints, and
	  each joint can have any number of degrees of freedom, which
	  is why getNJoints() and getNDOF() might come in handy, too.

	\todo Not sure whether RBDL would be able to support multi-DOF
	joints, so the proliferation of getNNodes, getNJoints, and
	getNDOF may be reduced to just one number.
    */
    size_t getNNodes() const;
    
    /** Compute or retrieve the cached number of joints in the
	robot. Note that each joint can have any number of degrees of
	freedom, which is why getNDOF() might come in handy, too. */
    size_t getNJoints() const;
    
    /** Compute or retrieve the cached number of degrees of freedom of
	the robot. Note that each joint can have any number of degrees
	of freedom, which is why this method might return something
	else than getNJoints(). */
    size_t getNDOF() const;
    
    /** Retrieve the name of a node. Returns an empty string in case
	the id is invalid. Use getNNodes() to find out how many nodes
	there are. */
    std::string getNodeName(size_t id) const;
    
    /** Retrieve the name of a joint. Returns an empty string in case
	the id is invalid. Use getNJoints() to find out how many
	joints there are.
	
	\todo Currently this just calls getNodeName because RBDL does
	not support joint names (yet). */
    std::string getJointName(size_t id) const { return getNodeName(id); }
    
    /** Retrieve a node by ID.
	
	\todo RBDL support in progress: this is a no-op kept to avoid
	too much code breakage.
    */
    size_t getNode(size_t id) const { return id; }
    
    /** Retrieve a node by its name.
	
	\todo Add support for registering aliases, such as
	"end-effector" or "End_Effector", which might correspond to
	"right-gripper" or so depending on the robot.
    */
    size_t getNodeByName(std::string const & name) const;
    
    /** Retrieve a node by joint name.  This will find and retrieve
	the node to which the joint is attached (see also
	getNodeByName()).
	
	\todo RBDL support in progress: RBDL has no joint names
	(yet... I may add them and send a pull request to the author)
	so I will use node names for now.
	
	\todo Add support for registering aliases, just as for
	getNodeByName().
    */
    size_t getNodeByJointName(std::string const & name) const { return getNodeByName(name); /* XXXX tmp refactoring */ }
    
    /** Retrieve joint limit information. This method fills the
	provided vectors with the lower and upper joint limits. In
	case no joint limit information is available, it sets the
	lower limit to \c std::numeric_limits<double>::min() and the
	upper limit to \c std::numeric_limits<double>::max().
	
	\todo RBDL has no support for joint limits (yet).
    */
    void getJointLimits(Vector & joint_limits_lower,
			Vector & joint_limits_upper) const;
    
    //////////////////////////////////////////////////
    // kinematic facet
    
    /** Computes the node origins wrt the global frame. */
    void updateKinematics();
    
    /** Retrieve the frame (translation and rotation) of a node
	origin. */
    void getGlobalFrame(size_t node,
			Transform & global_transform) const;
    
    /** Compute the global frame (translation and rotation)
	corresponding to a local frame expressed wrt the origin of a
	given node. */
    void computeGlobalFrame(size_t node,
			    Transform const & local_transform,
			    Transform & global_transform) const;
    
    /** Convenience method in case you are only interested in the
	translational part and hold the local point in three
	doubles. The computed global_transform will have the same
	rotational component as the node's origin. */
    void computeGlobalFrame(size_t node,
			    double local_x, double local_y, double local_z,
			    Transform & global_transform) const;
    
    /** Convenience method in case you are only interested in the
	translational part and hold the local point in a
	three-dimensional vector. The copmuted global_transform will
	have the same rotational component as the node's origin. */
    void computeGlobalFrame(size_t node,
			    Vector const & local_translation,
			    Transform & global_transform) const;
    
    void computeGlobalCOMFrame(size_t node,
			       Transform & global_com_transform) const;
    
    /** Compute the Jacobian (J_v over J_omega) at the origin of a
	given node. */
    void computeJacobian(size_t node,
			 Matrix & jacobian) const;
    
    /** Compute the Jacobian (J_v over J_omega) for a given node, at a
	point expressed wrt to the global frame.
	
	\todo Implement support for more than one joint per node, and
	more than one DOF per joint. */
    void computeJacobian(size_t node,
			 double gx, double gy, double gz,
			 Matrix & jacobian) const;
    
    /** Convenience method in case you are holding the global position
	in a three-dimensional vector. */
    inline void computeJacobian(size_t node,
				Vector const & global_point,
				Matrix & jacobian) const
    { return computeJacobian(node, global_point[0], global_point[1], global_point[2], jacobian); }
    
    //////////////////////////////////////////////////
    // dynamics facet
    
    /** Calls computeGravity(), computeCoriolisCentrifugal(),
	computeMassInertia(), and computeInverseMassInertia(). */
    void updateDynamics();
    
    /** Computes the location of the center of gravity, and optionally
	also its Jacobian. Pass opt_jcom=0 if you are not interested
	in the Jacobian. Failures can only be due to calls of
	computeJacobian() that happens for each node's contribution to
	the Jacobian of the COM. */
    void computeCOM(Vector & com, Matrix * opt_jcom) const;
    
    /** Compute the gravity joint-torque vector. */
    void computeGravity();
    
    /** Disable (or enable) gravity compensation for a given DOF
	(specified using its index). If you set \c disable to \c true,
	then getGravity() will knock out (set to zero) the
	corresponding entry of the gravity joint-torque vector.
	
	\note Invalid indices are silently ignored, and \c true is
	returned. Valid indices are \c 0<=index<getNDOF().
	
	\todo Double check on the range of indices employed by RBDL,
	conceivably it is 1<=index<=ndof instead.
	
	\return The previous value of \c disable for this joint. */
    bool disableGravityCompensation(size_t index, bool disable);
    
    /** Retrieve the gravity joint-torque vector.
	
	\pre You need to have called updateDynamics() (which gets
	called by update() too). */
    void getGravity(Vector & gravity) const;
    
    /** Compute the Coriolis and contrifugal joint-torque vector. If
	you set cc_tree=NULL in the constructor, then this is a
	no-op. */
    void computeCoriolisCentrifugal();
    
    /** Retrieve the Coriolis and contrifugal joint-torque vector.
	
	\pre You need to have called updateDynamics() (which gets                                          called by update() too). */
    void getCoriolisCentrifugal(Vector & coriolis_centrifugal) const;
    
    /** Compute the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix. */
    void computeMassInertia();
    
    /** Retrieve the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix.
	
	\pre You need to have called computeMassInertia(), or update()
	which calls updateDynamics() and that calls
	computeMassInertia() for you. */
    void getMassInertia(Matrix & mass_inertia) const;
    
    /** Compute the inverse joint-space mass-inertia matrix. */
    void computeInverseMassInertia();
    
    /** Retrieve the inverse joint-space mass-inertia matrix. 
	
	\pre You need to have called computeMassInertia(), or update()
	which calls updateDynamics() and that calls
	computeMassInertia() for you. */
    void getInverseMassInertia(Matrix & inverse_mass_inertia) const;
    
    RigidBodyDynamics::Model const * getRBDL() const { return rbdl_model_; }
    
    
  private:
    typedef std::set<size_t> dof_set_t;
    dof_set_t gravity_disabled_;
    
    RigidBodyDynamics::Model * rbdl_model_;
    
    State state_;
    Vector g_torque_;
    Vector cc_torque_;
    std::vector<double> a_upper_triangular_;
    std::vector<double> ainv_upper_triangular_;
  };
  
}

#endif // JSPACE_MODEL_HPP
