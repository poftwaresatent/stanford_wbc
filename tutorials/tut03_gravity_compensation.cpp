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
   \file tut03_gravity_compensation.cpp
   \author Roland Philippsen
   
   This tutorial demonstrates the effect of gravity compensation. It
   is very similar to the tutorial number 2 in that it uses a custom
   opspace::Task subclass that is just a simple joint-position PD
   servo. However, the tut03::JTask class contains a flag to switch
   gravity compensation on or off.
   
   When the tutorial starts, it is in a mode where the robot position
   and velocity gets constantly re-initialized to a swaying
   motion. When you press Toggle, it switches to joint-space position
   control with gravity compensation, using the current joint position
   as goal. The robot will thus overshoot due to its current velocity,
   and then converge back to the position it had when you clicked
   Toggle. Then, when you press Toggle again, it will keep servoing
   but switch off gravity compensation. This will make the robot "sag"
   with respect to its desired position, illustrating the effect of
   gravity. Clicking Toggle again goes back to the swaying motion and
   the cycle repeats.
   
*/

#include "tutsim.hpp"
#include <opspace/Task.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <err.h>


namespace tut03 {
  
  class JTask : public opspace::Task {
  public:
    JTask() : opspace::Task("tut03::JTask"), enable_gravity_compensation_(false) {}
    
    virtual jspace::Status init(jspace::Model const & model)
    {
      //////////////////////////////////////////////////
      // The Jacobian is not really required for pure jspace control,
      // but will become very important for operational-space control.
      
      jacobian_ = jspace::Matrix::Identity(model.getNDOF(), model.getNDOF());
      
      //////////////////////////////////////////////////
      // Initialize our PD parameters.
      
      kp_ = 100.0;
      kd_ = 20.0;
      
      //////////////////////////////////////////////////
      // Initialize our goal to the current configuration.
      
      goal_ = model.getState().position_;
      
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
      // later retrieval. If enabled, add the estimated effect of
      // gravity in order to make the robot behave as if was
      // weightless.
      
      command_ = kp_ * (goal_ - actual_) - kd_ * model.getState().velocity_;
      if (enable_gravity_compensation_) {
	jspace::Vector gg;
	if ( ! model.getGravity(gg)) {
	  return jspace::Status(false, "failed to retrieve gravity torque");
	}
	command_ += gg;
      }
      
      jspace::Status ok;
      return ok;
    }
    
    bool enable_gravity_compensation_;
    double kp_, kd_;
    jspace::Vector goal_;
  };
  
}


static std::string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<tut03::JTask> jtask;
static size_t mode(0);


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  mode = toggle_count % 3;
  static size_t prevmode(0);
  
  if (0 == mode) {
    
    //////////////////////////////////////////////////
    // Re-initialize simulator with a configuration so we can test the
    // task from various starting states.
    
    for (int ii(0); ii < state.position_.rows(); ++ii) {
      static double const amplitude(0.5 * M_PI);
      double const omega(1.0 + 0.1 * ii);
      double const phase(omega * 1e-3 * wall_time_ms);
      state.position_[ii] =         amplitude * sin(phase);
      state.velocity_[ii] = omega * amplitude * cos(phase);
    }
    prevmode = 0;
    return false;
    
  }
  
  //////////////////////////////////////////////////
  // Update the model to reflect the current robot state.
  
  model->update(state);
  
  //////////////////////////////////////////////////
  // Run the jtask, but re-initialize it whenever we start a new
  // cycle of trials, and switch gravity compensation on/off to
  // illustrate its effects.
  
  if (mode != prevmode) {
    if (0 == prevmode) {
      jtask->init(*model);
    }
    if (1 == mode) {
      jtask->enable_gravity_compensation_ = true;
    }
    else {
      jtask->enable_gravity_compensation_ = false;
    }
  }
  prevmode = mode;
  
  jtask->update(*model);
  command = jtask->getCommand();
  
  //////////////////////////////////////////////////
  // Print debug info from time to time.
  
  static size_t iteration(0);
  if (0 == (iteration % 100)) {
    switch (mode) {
    case 0:
      std::cerr << "mode: re-init simul\n";
      break;
    case 1:
      std::cerr << "mode: jtask with gravity compensation\n";
      jspace::pretty_print(jtask->goal_, std::cerr, "  goal", "    ");
      break;
    default:
      std::cerr << "mode: jtask without gravity compensation\n";
      jspace::pretty_print(jtask->goal_, std::cerr, "  goal", "    ");
      break;
    }
    jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
    jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");
    jspace::pretty_print(command, std::cerr, "  command", "    ");
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
    jtask.reset(new tut03::JTask());
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_draw_cb(draw_cb);
  return tutsim::run(servo_cb);
}
