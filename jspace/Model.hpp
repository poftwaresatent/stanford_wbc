#include <jspace/State.hpp>

// Clients of Model never really need to worry about what exactly lies
// behind TAO, they can treat this as an opaque pointer type.
class taoDNode;

class SAIVector;
class SAIMatrix;
class SAITransform;

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
	its nodes before computing the model. In order to avoid
	computing things that the user does not need, the Model
	internally uses a tick counter for caching: a given quantity
	is only computed if it is outdated, otherwise we simply
	retrieve the information already at hand. */
    void setState(State const & state);
    
    //////////////////////////////////////////////////
    // Bare tree accessors.
    //
    // We should actually distinguish between links and joints, but
    // the current parsers all ensure that there is exactly one joint
    // per link.
    
    int getNNodes();
    int getNJoints();
    taoDNode * getNode(int id);
    taoDNode * getNode(std::string const & name_or_alias);
    taoDNode * getJoint(std::string const & name_or_alias);
    int getNodeID(taoDNode const * node);
    
    //////////////////////////////////////////////////
    // kinematic facet
    
    /** Utility for computing local and global, forward and reverse
	frame transformations. from_node and to_node can be NULL, in
	which case the root (base) will be used. The result gets
	stored in the out_transform parameter, instead of being
	returned by value, in order to minimize the number of
	temporary copies in the system.
    */
    bool computeTransformation(taoDNode const * from_node,
			       taoDNode const * to_node,
			       SAITransform & out_transform);
    
    /** Jacobian at the origin of a node. */
    bool computeNodeJacobian(taoDNode const * node,
			     SAIMatrix & jacobian);
    
    /** Jacobian of a point that is attached to a given node. Passing
	a NULL frame means to interpret the point in the global
	frame. Use computeTransformation() for other cases. */
    bool computePointJacobian(taoDNode const * node,
			      SAITransform const * frame,
			      SAIVector const & point,
			      SAIMatrix & jacobian);
    
    //////////////////////////////////////////////////
    // dynamics facet
    
    /** Compute (or retrieve from the cache) the gravity joint-torque
	vector. */
    void getGravity(SAIVector & gravity) const;
    
    /** Compute (or retrieve from the cache) the Coriolis and
	contrifugal joint-torque vector. */
    void getCoriolisCentrifugal(SAIVector & coriolis_centrifugal) const;
    
    /** Compute (or retrieve from the cache) the joint-space
	mass-inertia matrix, a.k.a. the kinetic energy matrix. */
    void getMassInertia(SAIMatrix & mass_inertia) const;
    
    /** Computed (or retrieve from the cache) the inverse joint-space
	mass-inertia matrix. */
    void getInverseMassInertia(SAIMatrix & inverse_mass_inertia) const;
    
    
  private:
    // For the moment we are wrapping the existing code. Later we'll
    // cleanly migrate its joint-space-related parts to here. At that
    // point we can probably change these fields to protected instead
    // of private, to make it easier to extend this model.
    wbc::RobotControlModel * robmodel_;
    
    State state_;
    
    int state_tick_;
    int robmodel_tick_;
  };
  
}
