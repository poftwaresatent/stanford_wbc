/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen (Stanford) and Luis Sentis (UT Austin)
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
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

#ifndef OPSPACE_TASK_HPP
#define OPSPACE_TASK_HPP

#include <opspace/Parameter.hpp>
#include <jspace/Model.hpp>

namespace opspace {
  
  
  using jspace::Model;
  
  
  /**
     Partially abstract base class for all operational space
     tasks. The base class provides parameter introspection facilities
     and an interface that is used by opspace::Controller instances to
     come up with control signals for a robot.
     
     This is one of the most important classes in the opspace
     namespace. You implement tasks by subclassing from it (or one of
     its more specialized derivatives) and implementing the init() and
     update() methods. You can also override some other methods if you
     are not happy with the defaults.
     
     A simple but complete example of concrete Task subclass would
     look like this:
     
     \code
  class Thermostat : public Task {
  public:
    Thermostat(std::string const & name) : Task(name), temp_(0) {
      declareParameter("desired_temperature", &temp_);
    }
    
    virtual Status init(Model const & model) {
      if (1 != model.getNDOF()) {
	return Status(false, "are you sure this is a fridge?");
      }
      jacobian_ = Vector::Ones(1); // somewhat spurious in this example
      command_ = Vector::Zero(1);
      return update(model);
    }
    
    virtual Status update(Model const & model) {
      actual_ = model.getState().position_;
      if (actual_[0] > temp_) {
	command_[0] = 1;
      }
      else {
	command_[0] = 0;
      }
      // jacobian_ was set in init() and never changes in this example
      Status ok;
      return ok;
    }
    
  private:
    double temp_;
  };
     \endcode
     
  */
  class Task
    : public ParameterReflection
  {
  protected:
    explicit Task(std::string const & name);
    
  public:
    /**
       Abstract, implemented by subclasses in order to initialize the
       task. This is important for stateful tasks, for instance in
       order to initialize a trajectory-following behavior. The init()
       method also gets called when tasks are switched at runtime, so
       subclasses should NOT assume that init() only gets called once
       at startup.
    */
    virtual Status init(Model const & model) = 0;
    
    /**
       Abstract, implemented by subclasses in order to compute the
       current task state, the command acceleration, and the
       Jacobian. Given the current joint-space model passed as
       argument to this method, subclasses have to set the actual_,
       command_, and jacobian_ fields. These will then get retrieved
       according to the task hierarchy and assembled into joint torque
       commands using dynamically consistent nullspace projection.
       
       \note Make sure your subclass sets the actual_, command_, and
       jacobian_ fields in the implementation of this method.
    */
    virtual Status update(Model const & model) = 0;
    
    /**
       \return The actual "position" of the robot in this task
       space. Reminder: actual_ must be set by subclasses in their
       update() method.
    */
    Vector const & getActual() const   { return actual_; }
    
    /**
       \return The command of this task. In the operational space
       formulation, this is simply the desired acceleration of the
       task point (in task space, of course). Reminder: command_ must
       be set by subclasses in their update() method.
    */
    Vector const & getCommand() const  { return command_; }
    
    /**
       \return The current Jacobian of this task space. The Jacobian
       maps joint velocities to task space velocities (it thus has M
       rows and N columns, where M is the dimension of the task space
       and N the number of degrees of freedom of the robot). Usually,
       the Jacobian is configuration dependent and is updated along
       with the command in the update() method. Some tasks, however,
       have simple and constant Jacobians which can be set in the
       init() method. The Jacobian is used by the opspace::Controller
       to compensate for rigid body dynamics and to decouple tasks
       according to a strict hierarchy. Reminder: jacobian_ must be
       set by subclasses in their update() method.
    */
    Matrix const & getJacobian() const { return jacobian_; }

    /**
       SVD cutoff value for pseudo inverse, exists in all tasks
       because Controller implementations need it.
    */
    double getSigmaThreshold() const { return sigma_threshold_; }
    
    virtual void dump(std::ostream & os,
		      std::string const & title,
		      std::string const & prefix) const;
    
    virtual void dbg(std::ostream & os,
		     std::string const & title,
		     std::string const & prefix) const;
    
  protected:
    Vector actual_;
    Vector command_;
    Matrix jacobian_;
    
    /** Parameter "sigma_threshold", SVD cutoff value for pseudo
	inverse. Exists in all tasks because Controller
	implementations need it. */
    double sigma_threshold_;
  };
  
}

#endif // OPSPACE_TASK_HPP
