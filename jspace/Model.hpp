#include <jspace/State.hpp>

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
    explicit Model(wbc::RobotControlModel * robmodel);
    
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
    void getState(State & state) const;
    
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
    
    /** Retrieve the translation and rotation of a node origin. The
	translation is returned into a three-dimensional vector, and
	the rotation into a quaternion with elements x, y, z, and w.
	
	\note If you pass NULL as one of the \c vector_t parameters,
	that piece of information is discarded. For example, if you
	are only interested in a position, you can pass \c rotation=0,
	thus avoiding an unnecessary variable.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool getGlobalFrame(taoDNode const * node,
			vector_t * translation,
			vector_t * rotation) const;
    
    /** Compute the global translation and rotation corresponding to a
	frame expressed wrt the origin of a given node. The
	translation is returned into a three-dimensional vector, and
	the rotation into a quaternion with elements x, y, z, and w.
	
	\note If you pass NULL as one of the \c vector_t parameters,
	that piece of information is discarded. For example, if you
	are only interested in a position, you can pass \c rotation=0,
	thus avoiding an unnecessary variable. Similarly, if you can
	pass \c local_rotation=0.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    vector_t const * local_translation,
			    vector_t const * local_rotation,
			    vector_t * translation,
			    vector_t * rotation) const;
    
    /** Compute the global Jacobian for a point given in global
	coordinates. */
    void computeJacobian(vector_t const & global_point,
			 matrix_t & jacobian) const;
    
    /** Compute the global Jacobian at the origin of a given node.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeJacobian(taoDNode const * node,
			 matrix_t & jacobian) const;
    
    /** Compute the global Jacobian at a point expressed wrt to the
	origin of a given node.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeJacobian(taoDNode const * node,
			 vector_t const & local_point,
			 matrix_t & jacobian) const;
    
    //////////////////////////////////////////////////
    // dynamics facet
    
    /** Calls computeGravity(), computeCoriolisCentrifugal(),
	computeMassInertia(), and computeInverseMassInertia(). */
    void updateDynamics();
    
    /** Compute the gravity joint-torque vector. */
    void computeGravity();
    
    /** Retrieve the gravity joint-torque vector. */
    void getGravity(vector_t & gravity) const;
    
    /** Compute the Coriolis and contrifugal joint-torque vector. */
    void computeCoriolisCentrifugal();
    
    /** Retrieve the Coriolis and contrifugal joint-torque vector. */
    void getCoriolisCentrifugal(vector_t & coriolis_centrifugal) const;
    
    /** Compute the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix. */
    void computeMassInertia();
    
    /** Retrieve the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix. */
    void getMassInertia(matrix_t & mass_inertia) const;
    
    /** Compute the inverse joint-space mass-inertia matrix. */
    void computeInverseMassInertia();
    
    /** Retrieve the inverse joint-space mass-inertia matrix. */
    void getInverseMassInertia(matrix_t & inverse_mass_inertia) const;
    
    
  private:
    // For the moment we are wrapping the existing code. Later we'll
    // cleanly migrate its joint-space-related parts to here. At that
    // point we can probably change these fields to protected instead
    // of private, to make it easier to extend this model.
    wbc::RobotControlModel * robmodel_;
    
    State state_;
  };
  
}
