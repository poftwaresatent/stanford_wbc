#include <wbcnet/Factory.hpp>
// ...more includes etc to be filled in...


namespace wbc {
  
  /**
     This provides a semi-generic "service callback" interface for
     controller. See ../bin/directory.hpp for more info.
  */
  class BaseDirectoryCmdServer;
  
  
  /**
     Essentially "just" a TAO tree plus some syntactic sugar. Maybe we
     reuse BranchingRepresentation as-is, but it might be worthwhile
     to cleanly re-implement. The base model contains all the
     information from the robot specification (XML file) and provides
     data lookup methods, but no actual computations.
  */
  class BaseModel
  {
  public:
    taoDNode * findNode(std::string const & name_or_alias) const;
    taoDNode * getNode(int id) const;
  };
  
  
  /**
     To be further discussed. It is roughly based on the existing
     Kinematics.hpp, but simpler and with a cleaner functional
     interface (essentially just access transforms and Jacobians at
     points on the robot body).
     
     \note You can use BaseModel::findNode() and BaseModel::getNode()
     in order to lazily initialize any taoDNode pointers that your are
     likely to use in your controller.
     
     \todo Shall we introduce the notion of end-effector (or multiple
     nodes of interest), and an update() method that automatically
     computes the transformation(s) and Jacobian(s) thereof?
  */
  class JSpaceKinematicModel
    : public BaseModel
  {
  public:
    /** Utility for computing local and global, forward and reverse
	frame transformations. from_node and to_node can be NULL, in
	which case the root (base) will be used. The result gets
	stored in the out_transform parameter, instead of being
	returned by value, in order to minimize the number of
	temporary copies in the system.
	
	\todo Maybe allow optional local points?
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
  
  
  /**
     Roughly based on the current Dynamics.hpp, but simpler and
     limited to joint-space dynamics.
  */
  class JSpaceDynamicModel
    : public JSpaceKinematicModel
  {
  public:
    virtual ~JSpaceDynamicModel() {}
    
    /** Compute the gravity, Coriolis, and contrifugal torque vectors,
	the joint-space mass-inertia matrix, and its inverse. */
    virtual void update(SAIVector const & joint_angles,
			SAIVector const & joint_velocities);
    
    /** Retrieve the previously computed gravity torque vector. */
    void getGravity(SAIVector & gravity) const;

    /** Retrieve the previously computed Coriolis and contrifugal
	torque vector. */
    void getCoriolisCentrifugal(SAIVector & coriolis_centrifugal) const;
    
    /** Retrieve the previously computed joint-space mass-inertia
	matrix, a.k.a. the kinetic energy matrix. */
    void getMassInertia(SAIMatrix & mass_inertia) const;
    
    /** Retrieve the previously computed inverse of the joint-space
	mass-inertia matrix. */
    void getInverseMassInertia(SAIMatrix & inverse_mass_inertia) const;
  };
  
  
#ifdef SPACED_OUT_IDEAS

  /** The idea is to include both kinematic and dynamic aspects in the
      operational space model... but maybe that's not such a good
      idea. To be discussed. */
  class OpSpaceModel
    : public JSpaceDynamicModel {
  public:
    /** Essentially Lambda and LambdaStar, but should they be computed
	on-demand or inside the update() method? Consider that:
	
	1. Here is the point where we want to optionally support a
	separate model update process, for which it is more
	appropriate to implement getLambda() instead of
	computeLambda().
    
	2. Lambda is defined in op-space, thus it depends on the
	task... which we do not explicitly represent at this level. */
    virtual void update(SAIVector const & joint_angles,
			SAIVector const & joint_velocities);
  };
  
  
  class FreeFloatingModel
    : public OpSpaceModel {
  public:
    virtual void update(SAIVector const & joint_angles,
			SAIVector const & joint_velocities);
    
    /** Retrieve the previously computed center of mass.
	\todo This could move up the hierarchy to JSpaceDynamicModel,
	but in practice it is only relevant for robots that are free
	floating (e.g. humanoids) in order to maintain balance
	constraints. */
    void getCOM(SAIVector & com) const;
    
    /** Retrieve the previously computed Jacobian of the center of
	mass. */
    void getCOMJacobian(SAIMatrix & jacobian) const;
    
    /** Retrieve the previously computed zero-moment point. */
    void getZMP(SAIVector & zmp) const;
  };

#endif // SPACED_OUT_IDEAS
  
  
  class BaseControllerAPI
  {
  public:
    virtual ~BaseControllerAPI() {}
    
    /** Optionally, if you want to handle user process commands, you
	have to implement a subclass of BaseDirectoryCmdServer and
	here return a pointer to its instance. If you do not override
	this method, any command that the user process sends to your
	controller will simply bounce with a SRV_NOT_IMPLEMENTED
	status. */
    virtual BaseDirectoryCmdServer * getCmdServer() { return 0; }
  };
  
  
  /**
     The various controller levels are defined by the type of model
     they are using, from BaseModel to FreeFloatingModel. The
     templateized base class implementation handles some common chores
     for you:
     - you can optionally intercept service calls
     - the RobotAPI and networking stuff is handled for you
     
     You use this class, or any other of the provided semi-abstract
     hierarchy, by subclassing it in a plugin, registering a
     corresponding factory by name, and passing that name to the
     generic servo process.
     
     Depending on the model type, your controller can do more or less
     fancy things:
     
     - BaseModel: The "base" (or "raw") controller is pretty much the
       barest interface you can have to the robot. At each tick, it
       gets called with the current state, and your implementation
       computes the joint-space command torques, which then get passed
       to the robot.
       
     - JSpaceKinematicModel: Slightly more sophisticated than
       ControllerAPI<BaseModel>, here we add a kinematic model, which
       gets updated for you. You can use the kinematic model for
       computing global and local coordinate transforms of any point
       on the branching structure. Jacobians can also be
       computed. However, there is no notion of mass and inertia, not
       even for gravity compensation.
     
     - JSpaceDynamicModel: A controller conforming to
       ControllerAPI<JSpaceDynamicModel> has a notion of masses and
       inertias. It can take into accound gravity, Coriolis and
       centrifugal terms, and the joint-space mass-inertia matrix (and
       its inverse).
  */
  template<class ModelType_>
  class ControllerAPI
    : public BaseControllerAPI
  {
  public:
    typedef ModelType_ ModelType;
    
    /** This method gets called once, with a fully constructed but
	never updated model, before the first call to
	computeTorques(). You can override this method in order to
	perform custom initialization steps. The default
	implementation does nothing, and it might be easier to use
	lazy initialization, especially if you need the model to have
	been updated at least once. */
    virtual void init(ModelType const & model) throw(std::runtime_error) {}
    
    /** This is "the" method that gets called at each tick. The model
	gets updated for you before this call.
	
	\return False if a fatal error occurred, in which case the
	servo process aborts. You should use the logging mechanism to
	send out an information about the cause of the error. For
	simple cases, it might be sufficient to just write a message
	to stdout or stderr.
    */
    virtual bool computeTorques(SAIVector const & joint_angles,
				SAIVector const & joint_velocities,
				timeval const & acquisition_time,
				SAIMatrix const & contact_forces,
				ModelType const & model,
				SAIVector & command_torques) = 0;
    
  };
  
  
  /** Some glue for hooking ControllerAPI<> into the plugin factory
      mechanism... not quite sure yet whether this works:
      BaseControllerAPI has no computeTorques() method, because that
      would require the outside to be aware of its ModelType. However,
      we can always have a generic servo process that tries to
      dynamic_cast<> to one of the known subclasses of BaseModel. */
  template<class ModelType>
  class ControllerFactory
    : public wbcnet::Factory<ControllerAPI<ModelType>, BaseControllerAPI>
  {};
  
  
}
