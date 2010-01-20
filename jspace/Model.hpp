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
    explicit Model(wbc::RobotControlModel * robmodel);
    
    /** Inform the model about the joint state. We have to separate
	the state update from the computation of the various
	quantities in order to efficiently use the TAO tree
	representation, which forces us to distribute the state over
	its nodes before computing the model. */
    void setState(State const & state);
    
    //////////////////////////////////////////////////
    // Bare tree accessors.
    
    /** Compute or retrieve the cached number of links in the
	robot. Note that each link can have any number of joints, and
	each joint can have any number of degrees of freedom, which is
	why getNJoints() and getNDOF() might come in handy, too. */
    int getNLinks();
    
    /** Compute or retrieve the cached number of joints in the
	robot. Note that each joint can have any number of degrees of
	freedom, which is why getNDOF() might come in handy, too. */
    int getNJoints();
    
    /** Compute or retrieve the cached number of degrees of freedom of
	the robot. Note that each joint can have any number of degrees
	of freedom, which is why this method might return something
	else than getNJoints(). */
    int getNDOF();
    
    /** Retrieve a link by ID. */
    taoDNode * getLink(int id);
    
    /** Retrieve a link by its name or registered alias. A typical
	alias would be "end-effector" or "End_Effector" or so, which
	might correspond to "right-gripper" or so depending on the
	robot. */
    taoDNode * getLinkByName(std::string const & name_or_alias);
    
    /** Retrieve a link by a joint name or registered alias. This will
	find and retrieve the node to which the joint is attached (see
	also getLinkByName()), which allows you to retrieve the
	taoJoint instance itself. Note that a taoDNode can have any
	number of joints, so you might have to search through them to
	find the exact one you're looking for. */
    taoDNode * getLinkByJointName(std::string const & name_or_alias);
    
    //////////////////////////////////////////////////
    // kinematic facet
    
    /** Retrieve the translation and rotation of a node origin. The
	translation is returned into a three-dimensional vector, and
	the rotation into a quaternion with elements x, y, z, and w.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getLink() or one of
	the related methods you can safely ignore the return value. */
    bool getGlobalFrame(taoDNode const * node,
			vector_t translation,
			vector_t rotation);
    
    /** Compute the global Jacobian for a point given in global
	coordinates. */
    void computeJacobian(vector_t const & global_point,
			 matrix_t & jacobian);
    
    //////////////////////////////////////////////////
    // dynamics facet
    
    /** Retrieve the gravity joint-torque vector. */
    void getGravity(vector_t & gravity) const;
    
    /** Retrieve the Coriolis and contrifugal joint-torque vector. */
    void getCoriolisCentrifugal(vector_t & coriolis_centrifugal) const;
    
    /** Retrieve the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix. */
    void getMassInertia(matrix_t & mass_inertia) const;
    
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
