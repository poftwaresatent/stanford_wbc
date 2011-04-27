/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * Author: Roland Philippsen
 *         http://cs.stanford.edu/group/manips/
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
   \file tut02_jtask.cpp
   \author Roland Philippsen
   
   Example of how to create and use a custom subclass of
   opspace::Task. The tut02::JTask class performs joint-space PD
   control without compensating for either gravity or inertial
   coupling. It is a straightforward simplsitic implementation which
   serves to show the minimal implementation work required to create
   an opspace::Task, and it also demonstrates that without gravity and
   mass-inertia compensation you will get rather low performance.
   
   When you start the simulation, it is initially in a mode where it
   bypasses the tut02::JTask by simply sending a sinusoidal torque to
   the first joint and adding damping to all joints. This makes the
   arm sway around somewhat erratically. When you press Toggle, it
   sets the tut02::JTask goal to a position that is a zig-zag shape of
   +/- 45deg added to the current joint state, and it starts servoing
   to it. You'll see the arm converge more or less to the desired
   position. Pressing Toggle again goes back to the swaying mode.
*/

#include "tutsim.hpp"
#include <opspace/Task.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <err.h>


namespace tut02 {
  
  class JTask : public opspace::Task {
  public:
    JTask() : opspace::Task("tut02::JTask") {}
    
    virtual jspace::Status init(jspace::Model const & model)
    {
      //////////////////////////////////////////////////
      // The Jacobian is not really required for pure jspace control,
      // but will become very important for operational-space control.
      
      jacobian_ = jspace::Matrix::Identity(model.getNDOF(), model.getNDOF());
      
      //////////////////////////////////////////////////
      // Initialize our PD parameters.
      
      kp_ = 400.0;
      kd_ = 40.0;
      
      //////////////////////////////////////////////////
      // Initialize our goal to a configuration that is the current
      // one +/- 45 degrees on each joint.
      
      goal_ = model.getState().position_;
      for (int ii(0); ii < goal_.rows(); ++ii) {
	if (0 == (ii % 2)) {
	  goal_[ii] += M_PI / 4.0;
	}
	else {
	  goal_[ii] -= M_PI / 4.0;
	}
      }
      
      //////////////////////////////////////////////////
      // No initialization problems to report: the default constructor
      // of jspace::Status yields an instance that signifies success.
      
      jspace::Status ok;
      return ok;
    }
    
    
    virtual jspace::Status update(jspace::Model const & model)
    {
      //////////////////////////////////////////////////
      // Update the state of our task. Again, this is not critical
      // here, but important later when we want to integrate several
      // operational space tasks into a hierarchy.
      
      actual_ = model.getState().position_;
      
      //////////////////////////////////////////////////
      // Compute PD control torques and store them in command_ for
      // later retrieval.

      command_ = kp_ * (goal_ - actual_) - kd_ * model.getState().velocity_;
      
      jspace::Status ok;
      return ok;
    }
    
    double kp_, kd_;
    jspace::Vector goal_;
  };
  
}


static std::string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<tut02::JTask> jtask;
static size_t mode(0);


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  static size_t prev_toggle(1234);
  mode = toggle_count % 2;
  
  if (0 == mode) {
    
    //////////////////////////////////////////////////
    // Send torques that make the robot sway around.
    
    command = -3.0 * state.velocity_;
    command[0] = 40.0 * sin(1e-3 * sim_time_ms);
    
  }
  else {
    
    model->update(state);
    
    //////////////////////////////////////////////////
    // Use our JTask to compute the command, re-initializing it
    // whenever we switch here from the "swaying" mode, and setting
    // the goal to zero every other time.
    
    if (prev_toggle != toggle_count) {
      jtask->init(*model);
    }
    jtask->update(*model);
    command = jtask->getCommand();
    
  }
  
  prev_toggle = toggle_count;
  
  //////////////////////////////////////////////////
  // Print debug info from time to time.
  
  static size_t iteration(0);
  if (0 == (iteration % 100)) {
    std::cerr << "toggle: " << toggle_count << "  sim_time_ms: " << sim_time_ms << "\n";
    if (0 != (toggle_count % 2)) {
      jspace::pretty_print(jtask->goal_, std::cerr, "goal", "  ");
    }
    jspace::pretty_print(state.position_, std::cerr, "jpos", "  ");
    jspace::pretty_print(state.velocity_, std::cerr, "jvel", "  ");
    jspace::pretty_print(command, std::cerr, "command", "  ");
  }
  ++iteration;
  
  return true;
}


static void draw_cb(double x0, double y0, double scale)
{
  if (0 != mode) {
    tutsim::draw_robot(jtask->goal_, 2, 100, 80, 80, x0, y0, scale);
    tutsim::draw_delta_jpos(jtask->goal_, 1, 120, 120, 80, x0, y0, scale);
  }
}


int main(int argc, char ** argv)
{
  try {
    model.reset(jspace::test::parse_sai_xml_file(model_filename, true));
    jtask.reset(new tut02::JTask());
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_draw_cb(draw_cb);
  return tutsim::run(servo_cb);
}
