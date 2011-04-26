/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *                    Author: Roland Philippsen
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
