namespace wbc {
  
  /** Essentially "just" a TAO tree plus some syntactic sugar.  We
      might replace it by something more lightweight... */
  class RobotControlModel;
  
  /** This provides a semi-generic "service callback" interface for
      controller. See ../bin/directory.hpp for more info. */
  class BaseDirectoryCmdServer;
  
  
  /** To be further discussed. It will be roughly based on the
      existing Kinematics.hpp, but simpler and with a cleaner
      interface. Essentially, only access to transforms and Jacobians
      at points on the robot body. */
  class JSpaceKinematics {
  public:
    /** Utility for computing local and global, forward and reverse
	frame transformations. from_node and to_node can be NULL, in
	which case the root (base) will be used. The result gets
	stored in the out_transform parameter, instead of being
	returned by value, in order to minimize the number of
	temporary copies in the system.
	
	\todo
	- maybe better to use node IDs... or provide wrapper functions
	- maybe allow optional local points...
    */
    static bool computeTransformation(taoDNode const * from_node,
				      taoDNode const * to_node,
				      SAITransform & out_transform);
    
    /** Jacobian at the origin of a node. */
    static bool computeJacobian(taoDNode const * node,
				SAIMatrix & jacobian);
    
    /** Jacobian of a point that is given wrt to the global
	(base/root) frame. */
    static bool computeJacobianGlobal(taoDNode const * node,
				      SAIVector const & global_point,
				      SAIMatrix & jacobian);
    
    /** Jacobian of a point that is given wrt to the frame of the
	node. */
    static bool computeJacobianLocal(taoDNode const * node,
				     SAIVector const & local_point,
				     SAIMatrix & jacobian);
  };
  
  
  /** Roughly based on the current Dynamics.hpp, but simpler and
      limited to joint-space dynamics. */
  class JSpaceDynamics {
  public:
    void getGravity(SAIMatrix & gravity) const;
    void getCoriolisCentrifugal(SAIMatrix & coriolisCentrifugal) const;
    void getMassInertia(SAIMatrix & massInertia) const;
    void getInverseMassInertia(SAIMatrix & inverseMassInertia) const;
  };
  
  
  /** The idea is to include both kinematic and dynamic aspects in the
      operational space model... but maybe that's not such a good
      idea. To be discussed. */
  class OpSpaceModel;
  
  
  /**
     The "base" (or "raw") controller is pretty much the barest
     interface you can have to the robot. At each tick, it gets called
     with the current state, and your implementation computes the
     joint-space command torques, which then get passed to the
     robot. The only abstractions you are taking advantage of at this
     level are:
     - you can optionally intercept service calls
     - the RobotAPI and networking stuff is handled for you
     
     You use this class, or any other of the provided semi-abstract
     hierarchy, by subclassing it in a plugin, registering a
     corresponding factory by name, and passing that name to the
     generic servo process.
  */
  class BaseController
  {
  public:
    virtual ~BaseController() {}
    
    /** Optionally, if you want to handle user process commands, you
	have to implement a subclass of BaseDirectoryCmdServer and
	return a pointer to its instance here. If you do not override
	this method, any command that the user process sends to your
	controller will simply bounce with a SRV_NOT_IMPLEMENTED
	status. */
    virtual BaseDirectoryCmdServer * getCmdServer() { return 0; }
    
    /** This is "the" method that gets called at each tick. */
    virtual bool computeTorques(SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				SAIVector & command_torques) = 0;
  };
  
  
  /**
     Slightly more sophisticated than the BaseController, here we add
     a kinematic model, which gets updated for you. You can use the
     kinematic model for computing global and local coordinate
     transforms of any point on the branching structure. Jacobians can
     also be computed. However, there is no notion of mass and
     inertia, not even for gravity compensation.
  */
  class JSpaceKinematicController
    : public BaseController
  {
  public:
    /** This base class needs to be initialized with a robot model,
	such that it can update the kinematics for you. The exact
	semantics of this method might still change, but as long as
	you only override the abstract computeTorques() method you
	should not be concerned with such changes. */
    bool init(RobotControlModel const & robot_model);
    
    /** Updates the kinematic model based on the current state, then
	forwards the call to the more specialized (and abstract)
	computeTorques() method, which has to be provided by subclass
	implementers. */
    virtual bool computeTorques(SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				SAIVector & command_torques);
    
    /** This is "the" method that needs to implementated in order to
	provide a joint-space kinematic controller. */
    virtual bool computeTorques(SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				JSpaceKinematics const & kinematics,
				SAIVector & command_torques) = 0;
    
  protected:
    RobotControlModel const & robot_model_;
  };
  
  
  /**
     In addition to the kinematics, this controller base class also
     updates the dynamics for you:
     - gravity, Coriolis, and centrifugal torque vectors
     - center of mass and its Jacobian
     - mass-inertia matrix and its inverse
  */
  class JSpaceDynamicController
    : public JSpaceKinematicController
  {
  public:
    virtual bool computeTorques(SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				JSpaceKinematics const & kinematics,
				SAIVector & command_torques);

    virtual bool computeTorques(SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				JSpaceKinematics const & kinematics,
				JSpaceDynamics const & dynamics,
				SAIVector & command_torques) = 0;

  };
  
  
}
