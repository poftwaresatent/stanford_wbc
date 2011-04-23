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

// Clients of Model never really need to worry about what exactly lies
// behind TAO, they can treat this as an opaque pointer type.
class taoDNode;
class taoJoint;

namespace jspace {
  
  
  // declared in <jspace/tao_util.hpp>
  struct tao_tree_info_s;
  
  class Model
  {
  public:
    /** Please use the init() method in order to initialize your
	jspace::Model. It does some sanity checking, and error
	handling from within a constructor is just not so great.
    */
    Model();
    
    ~Model();
    
    /** Initialize the model with a TAO tree. Actually, it needs two
	copies of the same tree if you want to estimate centrifugal
	and Coriolis effects, but that is optional.
	
	This method also does some sanity checks and will return a
	non-zero error code if something is amiss. In order to get
	human-readable error strings, just pass a non-NULL msg pointer
	in as well. For instance, &std::cout will do nicely in most
	cases.
	
	\note Transfers ownership of the given TAO trees. They will be
	deleted when this jspace::Model instance is destructed. Also
	note that their info vector might get reordered in order to
	ensure that each node sits at the index that corresponds to
	its ID.
	
	\return 0 on success.
    */
    int init(/** TAO tree info used for computing kinematics, the
		 gravity torque vector, the mass-inertia matrix, and
		 its inverse. This tree will be deleted in the
		 jspace::Model destructor. */
	     tao_tree_info_s * kgm_tree,
	     /** Optional TAO tree info for computing Coriolis and
		 centrifugal torques. If you set this to NULL, then
		 the Coriolis and centrifugal forces won't be
		 computed. This tree will be deleted in the
		 jspace::Model destructor. */
	     tao_tree_info_s * cc_tree,
	     /** Optional stream that will receive error messages from
		 the consistency checks. */
	     std::ostream * msg);
    
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
    
    /** Inform the model about the joint state. We have to separate
	the state update from the computation of the various
	quantities in order to efficiently use the TAO tree
	representation, which forces us to distribute the state over
	its nodes before computing the model.
	
	\note The given state has to have the correct dimensions, but
	this is not checked by the implementation. If the given state
	has too few dimensions, then some positions and velocities of
	the model will remain at their old values. If there are too
	many dimensions, they will be ignored.
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
	  is why getNJoints() and getNDOF() might come in handy, too. */
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
	
	\todo A joint can have any number of DOF, which means there
	should be a way to get at them individually, but currently we
	only support exactly one 1-DOF joints per node. */
    std::string getJointName(size_t id) const;
    
    /** Retrieve a node by ID. */
    taoDNode * getNode(size_t id) const;
    
    /** Retrieve a node by its name.
	
	\todo Add support for registering aliases, such as
	"end-effector" or "End_Effector", which might correspond to
	"right-gripper" or so depending on the robot.
    */
    taoDNode * getNodeByName(std::string const & name) const;
    
    /** Retrieve a node by joint name.  This will find and retrieve
	the node to which the joint is attached (see also
	getNodeByName()), which allows you to retrieve the taoJoint
	instance itself.
	
	\note In principle, a taoDNode can have any number of joints,
	so you might have to search through them to find the exact one
	you're looking for. However, all use cases so far seem to be
	limited to exactly one joint per node (and each node having
	exactly one joint).
	
	\todo Add support for registering aliases, just as for getNodeByName().
    */
    taoDNode * getNodeByJointName(std::string const & name) const;
    
    /** Retrieve joint limit information. This method fills the
	provided vectors with the lower and upper joint limits. In
	case no joint limit information is available, it sets the
	lower limit to \c std::numeric_limits<double>::min() and the
	upper limit to \c std::numeric_limits<double>::max(). */
    void getJointLimits(Vector & joint_limits_lower,
			Vector & joint_limits_upper) const;
    
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
			Transform & global_transform) const;
    
    /** Compute the global frame (translation and rotation)
	corresponding to a local frame expressed wrt the origin of a
	given node.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    Transform const & local_transform,
			    Transform & global_transform) const;
    
    /** Convenience method in case you are only interested in the
	translational part and hold the local point in three
	doubles. The copmuted global_transform will have the same
	rotational component as the node's origin.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    double local_x, double local_y, double local_z,
			    Transform & global_transform) const;
    
    /** Convenience method in case you are only interested in the
	translational part and hold the local point in a
	three-dimensional vector. The copmuted global_transform will
	have the same rotational component as the node's origin.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    Vector const & local_translation,
			    Transform & global_transform) const;
    
    bool computeGlobalCOMFrame(taoDNode const * node,
			       Transform & global_com_transform) const;
    
    /** Compute the Jacobian (J_v over J_omega) at the origin of a
	given node.
	
	\note This just ends up calling the other computeJacobian()
	which takes a global point as argument, passing in the origin
	of the given node.
	
	\return True on success. There are two possible failures: an
	invalid node, or an unsupported joint type. If you got the
	node using getNode() or one of the related methods, then you
	need to extend this implementation when it returns false. */
    bool computeJacobian(taoDNode const * node,
			 Matrix & jacobian) const;
    
    /** Compute the Jacobian (J_v over J_omega) for a given node, at a
	point expressed wrt to the global frame.
	
	\todo Implement support for more than one joint per node, and
	more than one DOF per joint.
	
	\return True on success. There are two possible failures: an
	invalid node, or an unsupported joint type. If you got the
	node using getNode() or one of the related methods, then you
	need to extend this implementation when it returns false. */
    bool computeJacobian(taoDNode const * node,
			 double gx, double gy, double gz,
			 Matrix & jacobian) const;
    
    /** Convenience method in case you are holding the global position
	in a three-dimensional vector. */
    inline bool computeJacobian(taoDNode const * node,
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
    bool computeCOM(Vector & com, Matrix * opt_jcom) const;
    
    /** Compute the gravity joint-torque vector. */
    void computeGravity();
    
    /** Disable (or enable) gravity compensation for a given DOF
	(specified using its index). If you set \c disable to \c true,
	then getGravity() will knock out (set to zero) the
	corresponding entry of the gravity joint-torque vector.
	
	\note Invalid indices are silently ignore, and \c true is
	returned. Valid indices are \c 0<=index<getNDOF().
	
	\return The previous value of \c disable for this joint. */
    bool disableGravityCompensation(size_t index, bool disable);
    
    /** Retrieve the gravity joint-torque vector.
	
	\return True on success. The only possibility of receiving
	false is if you never called updateDynamics(), which gets
	called by updateDynamics(), which gets called by update(). */
    bool getGravity(Vector & gravity) const;
    
    /** Compute the Coriolis and contrifugal joint-torque vector. If
	you set cc_tree=NULL in the constructor, then this is a
	no-op. */
    void computeCoriolisCentrifugal();
    
    /** Retrieve the Coriolis and contrifugal joint-torque vector.
	
	\return True on success. There are two possibility of
	receiving false: (i) you set cc_tree=NULL in the constructor,
	or (ii) you never called computeCoriolisCentrifugal(), which
	gets called by updateDynamics(), which gets called by
	update(). */
    bool getCoriolisCentrifugal(Vector & coriolis_centrifugal) const;
    
    /** Compute the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix. */
    void computeMassInertia();
    
    /** Retrieve the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix.
	
	\return True on success. The only possibility of receiving
	false is if you never called computeMassInertia(), which gets
	called by updateDynamics(), which gets called by update(). */
    bool getMassInertia(Matrix & mass_inertia) const;
    
    /** Compute the inverse joint-space mass-inertia matrix. */
    void computeInverseMassInertia();
    
    /** Retrieve the inverse joint-space mass-inertia matrix. 
	
	\return True on success. The only possibility of receiving
	false is if you never called computeMassInertia(), which gets
	called by updateDynamics(), which gets called by update(). */
    bool getInverseMassInertia(Matrix & inverse_mass_inertia) const;
    
    
    /** For debugging only, access to the
	kinematics-gravity-mass-inertia tree. */
    tao_tree_info_s * _getKGMTree() { return kgm_tree_; }
    
    /** For debugging only, access to the optional
	Coriolis-centrifugal tree. Can NULL if the user is not
	interested in Coriolis-centrifugal effects. */
    tao_tree_info_s * _getCCTree() { return cc_tree_; }
    
    
  private:
    typedef std::set<size_t> dof_set_t;
    dof_set_t gravity_disabled_;
    
    std::size_t ndof_;
    tao_tree_info_s * kgm_tree_;
    tao_tree_info_s * cc_tree_;
    
    State state_;
    Vector g_torque_;
    Vector cc_torque_;
    std::vector<double> a_upper_triangular_;
    std::vector<double> ainv_upper_triangular_;
    
    struct ancestry_entry_s {
      int id;
      taoJoint * joint;
    };
    typedef std::list<ancestry_entry_s> ancestry_list_t;
    typedef std::map<taoDNode *, ancestry_list_t> ancestry_table_t;
    ancestry_table_t ancestry_table_;
  };
  
}

#endif // JSPACE_MODEL_HPP
