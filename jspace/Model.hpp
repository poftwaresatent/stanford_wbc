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
   \file jspace/Model.hpp
   \author Roland Philippsen, inspired by wbc/core code of Luis Sentis
*/

#include <jspace/State.hpp>
#include <saimatrix/SAITransform.h>

// Clients of Model never really need to worry about what exactly lies
// behind TAO, they can treat this as an opaque pointer type.
class taoDNode;

namespace wbc {
  // For the moment we are wrapping the existing code. Later we'll
  // cleanly migrate its joint-space-related parts to here.
  class RobotControlModel;
}

namespace jspace {
  
  
  class Model
  {
  public:
    /** For the moment we are wrapping the existing code. Later we'll
	cleanly reimplement it with direct access to TAO. */
    Model(/** The existing robot model that we are wrapping. */
	  wbc::RobotControlModel * robmodel,
	  /** If true, then the jspace::Model destructor deletes the
	      robmodel. Otherwise, it is left alone. */
	  bool cleanup_robmodel);
    
    ~Model();
    
    //////////////////////////////////////////////////
    // fire-and-forget facet
    
    /** Call setState(), updateKinematics(), and
	updateDynamics(). After calling the update() method, you can
	use any of the other methods without worrying whether you have
	already called the corresponding computeFoo() method. */
    void update(State const & state);
    
    /** Inform the model about the joint state. We have to separate
	the state update from the computation of the various
	quantities in order to efficiently use the TAO tree
	representation, which forces us to distribute the state over
	its nodes before computing the model. */
    void setState(State const & state);
    
    /** Retrieve the state passed to setState() (or update(), for that
	matter). */
    inline State const & getState() const { return state_; }
    
    //////////////////////////////////////////////////
    // Bare tree accessors.
    
    /** Compute or retrieve the cached number of nodes in the
	robot. Note that each node can have any number of joints, and
	each joint can have any number of degrees of freedom, which is
	why getNJoints() and getNDOF() might come in handy, too. */
    int getNNodes() const;
    
    /** Compute or retrieve the cached number of joints in the
	robot. Note that each joint can have any number of degrees of
	freedom, which is why getNDOF() might come in handy, too. */
    int getNJoints() const;
    
    /** Compute or retrieve the cached number of degrees of freedom of
	the robot. Note that each joint can have any number of degrees
	of freedom, which is why this method might return something
	else than getNJoints(). */
    int getNDOF() const;
    
    /** Retrieve a node by ID. */
    taoDNode * getNode(int id) const;
    
    /** Retrieve a node by its name or registered alias. A typical
	alias would be "end-effector" or "End_Effector" or so, which
	might correspond to "right-gripper" or so depending on the
	robot. */
    taoDNode * getNodeByName(std::string const & name_or_alias) const;
    
    /** Retrieve a node by a joint name or registered alias. This will
	find and retrieve the node to which the joint is attached (see
	also getNodeByName()), which allows you to retrieve the
	taoJoint instance itself. Note that a taoDNode can have any
	number of joints, so you might have to search through them to
	find the exact one you're looking for. */
    taoDNode * getNodeByJointName(std::string const & name_or_alias) const;
    
    //////////////////////////////////////////////////
    // kinematic facet
    
    /** Computes the node origins wrt the global frame. */
    void updateKinematics();
    
    /** Retrieve the frame (translation and rotation) of a node
	origin.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool getGlobalFrame(taoDNode const * node,
			SAITransform & global_transform) const;
    
    /** Compute the global frame (translation and rotation)
	corresponding to a local frame expressed wrt the origin of a
	given node.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    SAITransform const & local_transform,
			    SAITransform & global_transform) const;
    
    /** Compute the Jacobian (J_v over J_omega) at the origin of a
	given node.
	
	\todo Reimplement this "properly" using the explicit form.
	
	\return True on success. There are two possible failures: an
	invalid node, or an unsupported joint type. If you got the
	node using getNode() or one of the related methods, then you
	need to extend this implementation when it returns false. */
    bool computeJacobian(taoDNode const * node,
			 SAIMatrix & jacobian) const;
    
    /** Compute the Jacobian (J_v over J_omega) for a given node, at a
	point expressed wrt to the global frame. If you have the
	expression of the point in the local frame, call
	computeGlobalFrame() and use the translational part of the
	resulting frame.
	
	\todo Reimplement this "properly" using the explicit form.
	
	\return True on success. There are two possible failures: an
	invalid node, or an unsupported joint type. If you got the
	node using getNode() or one of the related methods, then you
	need to extend this implementation when it returns false. */
    bool computeJacobian(taoDNode const * node,
			 SAIVector const & global_point,
			 SAIMatrix & jacobian) const;
    
    //////////////////////////////////////////////////
    // dynamics facet
    
    /** Calls computeGravity(), computeCoriolisCentrifugal(),
	computeMassInertia(), and computeInverseMassInertia().
	
	\todo At the moment, this just calls onUpdate() on the wrapped
	wbc::Dynamics instance.
    */
    void updateDynamics();
    
    /** Compute the gravity joint-torque vector.
	
	\todo At the moment, this is a no-op because
	updateDynamics() actually does it for us. */
    void computeGravity();
    
    /** Retrieve the gravity joint-torque vector. */
    void getGravity(SAIVector & gravity) const;
    
    /** Compute the Coriolis and contrifugal joint-torque vector.
	
	\todo At the moment, this is a no-op because
	updateDynamics() actually does it for us. */
    void computeCoriolisCentrifugal();
    
    /** Retrieve the Coriolis and contrifugal joint-torque vector. */
    void getCoriolisCentrifugal(SAIVector & coriolis_centrifugal) const;
    
    /** Compute the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix.
	
	\todo At the moment, this is a no-op because
	updateDynamics() actually does it for us. */
    void computeMassInertia();
    
    /** Retrieve the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix. */
    void getMassInertia(SAIMatrix & mass_inertia) const;
    
    /** Compute the inverse joint-space mass-inertia matrix.
	
	\todo At the moment, this is a no-op because
	updateDynamics() actually does it for us. */
    void computeInverseMassInertia();
    
    /** Retrieve the inverse joint-space mass-inertia matrix. */
    void getInverseMassInertia(SAIMatrix & inverse_mass_inertia) const;
    
    
  private:
    // For the moment we are wrapping the existing code. Later we'll
    // cleanly migrate its joint-space-related parts to here. At that
    // point we can probably change these fields to protected instead
    // of private, to make it easier to extend this model.
    wbc::RobotControlModel * robmodel_;
    bool cleanup_robmodel_;
    
    State state_;
  };
  
}
