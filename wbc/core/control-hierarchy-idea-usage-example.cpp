#include "control-hierarchy-idea.hpp"
#include <wbc/core/Plugin.hpp>


/** Example controller that could use e.g. Jacobian-transpose to drive
    the end-effector to a desired location. */
class MyKinematicController
  : public wbc::ControllerAPI<wbc::JSpaceKinematicModel>
{
public:
  MyKinematicController()
    : end_effector_(0)
  {
  }
  
  /** Example init() method that verifies the existance of an
      end-effector. Note that ModelType is typedef-ed to
      wbc::JSpaceKinematicModel in our base class. */
  virtual void init(ModelType const & model) throw(std::runtime_error)
  {
    end_effector_ = model.findNode("End_Effector");
    if ( ! end_effecor_) {
      throw std::runtime_error("MyKinematicController::init(): no End_Effector node");
    }
    
    //////////////////////////////////////////////////
    // initialize other stuff, such as PID instances...
    //////////////////////////////////////////////////
    
  }
  
  /** Example computeTorques() method that */
  virtual bool computeTorques(SAIVector const & joint_angles,
			      SAIVector const & joint_velocities,
			      timeval const & acquisition_time,
			      SAIMatrix const & contact_forces,
			      ModelType const & model,
			      SAIVector & command_torques)
  {
    if ( ! model.computeJacobian(end_effector_, jacobian_)) {
      std::cerr << "ERROR in MyKinematicController::computeTorques(): could not compute Jacobian at end effector\n";
      return false;
    }
    
    //////////////////////////////////////////////////
    // use the Jacobian and other stuff (PID instances) to compute torques...
    //////////////////////////////////////////////////
    
    return true;
  }
    
protected:
  taoDNode * end_effector_;
  SAIMatrix jacobian_;
};


class MyControllerPlugin: public wbc::Plugin {
public:
  virtual void Init(wbc::Extensions & extensions) throw(std::runtime_error)
  {
    extensions.AddController("MyController", new wbc::ControllerFactory<MyController>());
  }
};


wbcnet::Module * wbcnet_create_module()
{
  return new MyControllerPlugin();
}
