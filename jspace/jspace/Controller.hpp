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
   \file jspace/Controller.hpp
   \author Roland Philippsen
*/

#ifndef JSPACE_CONTROLLER_HPP
#define JSPACE_CONTROLLER_HPP

#include <jspace/Status.hpp>
#include <jspace/wrap_eigen.hpp>
#include <vector>


namespace jspace {
  
  class Model;
  
  
  /** Interface for retrieving custom info (DOF names etc) from
      controllers. */
  struct controller_info_getter_s {
    virtual ~controller_info_getter_s() {}
    virtual void getDOFNames(Model const & model, std::vector<std::string> & names) const = 0;
    virtual void getDOFUnits(Model const & model, std::vector<std::string> & names) const = 0;
    virtual void getGainNames(Model const & model, std::vector<std::string> & names) const = 0;
    virtual void getLimits(Model const & model, Vector & limits_lower, Vector & limits_upper) const = 0;
  };
  
  /** Default info getter is based on the jspace::Model. */
  struct jspace_controller_info_getter_s
    : public controller_info_getter_s {
    virtual void getDOFNames(Model const & model, std::vector<std::string> & names) const;
    virtual void getDOFUnits(Model const & model, std::vector<std::string> & names) const;
    virtual void getGainNames(Model const & model, std::vector<std::string> & names) const;
    virtual void getLimits(Model const & model, Vector & limits_lower, Vector & limits_upper) const;
  };
  
  
  class Controller
  {
  public:
    Controller();
    virtual ~Controller();
    
    /** Provides a hook for retrieving specialized DOF and gain names,
	if provided. Specific Controller subclasses can override this
	method in order to inform higher levels about how to refer to
	the degrees of freedom etc. Useful mostly for operational
	space controllers.
	
	\note The default is to construct a
	jspace_controller_info_getter_s on the fly and return
	that. The Controller destructor will clean it up.
	
	\return A pointer to a (custom) controller_info_getter_s or
	NULL (i.e. we cannot construct the info getter before init()
	has been called).
    */
    virtual controller_info_getter_s const * getInfo() const;
    
    /** Default init just returns ok. You should only call this with a
	fully initialized model, because controllers might need to
	inspect the state. */
    virtual Status init(Model const & model) { Status ok; return ok; }
    
    virtual Status setGoal(Vector const & goal) = 0;
    virtual Status getGoal(Vector & goal) const = 0;
    virtual Status getActual(Vector & actual) const = 0;
    
    virtual Status setGains(Vector const & kp, Vector const & kd) = 0;
    virtual Status getGains(Vector & kp, Vector & kd) const = 0;
    
    /** This method is supposed to be called just before the first
	call to computeCommand(), in order to allow the controller to
	set the goal to the current state. This is important for
	applications where we switch controllers at runtime. */
    virtual Status latch(Model const & model) = 0;
    
    virtual Status computeCommand(Model const & model, Vector & tau) = 0;
    
  protected:
    mutable jspace_controller_info_getter_s * info_getter_;
  };
  
}

#endif // JSPACE_CONTROLLER_HPP
