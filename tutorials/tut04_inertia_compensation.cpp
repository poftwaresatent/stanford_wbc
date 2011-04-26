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


namespace tut04 {
  
  class JTask : public opspace::Task {
  public:
    typedef enum {
      COMPENSATION_FULL,
      COMPENSATION_DIAGONAL,
      COMPENSATION_OFF
    } compensation_mode_t;
    
    JTask()
      : opspace::Task("tut04::JTask"),
	initialized_(false),
	compensation_mode_(COMPENSATION_FULL) {}
    
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
      // Initialize our goal to the current state.
      
      goalpos_ = model.getState().position_;
      goalvel_ = model.getState().velocity_;
      
      //////////////////////////////////////////////////
      // No initialization problems to report: the default constructor
      // of jspace::Status yields an instance that signifies success.
      
      initialized_ = true;
      jspace::Status ok;
      return ok;
    }
    
    
    virtual jspace::Status update(jspace::Model const & model)
    {
      //////////////////////////////////////////////////
      // Lazy init...
      
      if ( ! initialized_ ) {
	init(model);
      }
      
      //////////////////////////////////////////////////
      // Update the state of our task. Again, this is not critical
      // here, but important later when we want to integrate several
      // operational space tasks into a hierarchy.
      
      actual_ = model.getState().position_;
      
      //////////////////////////////////////////////////
      // Compute PD control torques and store them in command_ for
      // later retrieval. Add feed-forward inertia decoupling terms as
      // requested by compensation_mode_. Gravity compensation is
      // always on.
      
      jspace::Vector gamma;
      gamma = kp_ * (goalpos_ - actual_) + kd_ * (goalvel_ - model.getState().velocity_);
      
      if (COMPENSATION_OFF == compensation_mode_) {
	command_ = gamma;
      }
      else {
	jspace::Matrix aa;
	if ( ! model.getMassInertia(aa)) {
	  return jspace::Status(false, "failed to retrieve inertia");
	}
	if (COMPENSATION_FULL == compensation_mode_) {
	  command_ = aa * gamma;
	}
	else {
	  command_.resize(gamma.rows());
	  for (int ii(0); ii < gamma.rows(); ++ii) {
	    command_[ii] = aa.coeff(ii, ii) * gamma[ii];
	  }
	}
      }
      
      jspace::Vector gg;
      if ( ! model.getGravity(gg)) {
	return jspace::Status(false, "failed to retrieve gravity torque");
      }
      command_ += gg;
      
      jspace::Status ok;
      return ok;
    }

    bool initialized_;    
    compensation_mode_t compensation_mode_;    
    double kp_, kd_;
    jspace::Vector goalpos_, goalvel_;
  };
  
}


static std::string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<tut04::JTask> jtask;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  size_t const mode(toggle_count % 4);
  
  //////////////////////////////////////////////////
  // Compute goal position and velocity to follow a sinusoidal joint
  // space motion.
  
  jtask->goalpos_.resize(state.position_.rows());
  jtask->goalvel_.resize(state.position_.rows());
  for (int ii(0); ii < state.position_.rows(); ++ii) {
    static double const amplitude(0.9 * M_PI);
    double const omega(0.5 + 0.3 * ii);
    double const phase(omega * 1e-3 * wall_time_ms);
    jtask->goalpos_[ii] =         amplitude * sin(phase);
    jtask->goalvel_[ii] = omega * amplitude * cos(phase);
  }
  
  static size_t iteration(0);
  
  if (0 == mode) {
    
    //////////////////////////////////////////////////
    // Re-initialize simulator to current goal position and velocity.
    
    state.position_ = jtask->goalpos_;
    state.velocity_ = jtask->goalvel_;
    
    if (0 == (iteration % 100)) {
      std::cerr << "re-init simulation to:\n";
      jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
      jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");
    }
    
    ++iteration;
    return false;
    
  }
  
  //////////////////////////////////////////////////
  // Update the model to reflect the current robot state.
  
  model->update(state);
  
  //////////////////////////////////////////////////
  // Cycle through the various inertia compensation modes.
  
  switch (mode) {
  case 1:
    jtask->compensation_mode_ = tut04::JTask::COMPENSATION_FULL;
    break;
  case 2:
    jtask->compensation_mode_ = tut04::JTask::COMPENSATION_DIAGONAL;
    break;
  default:
    jtask->compensation_mode_ = tut04::JTask::COMPENSATION_OFF;
  }
  
  //////////////////////////////////////////////////
  // Compute the command using jtask... as usual.
  
  jtask->update(*model);
  command = jtask->getCommand();
  
  //////////////////////////////////////////////////
  // Print debug info from time to time.
  
  if (0 == (iteration % 100)) {
    switch (jtask->compensation_mode_) {
    case tut04::JTask::COMPENSATION_FULL:
      std::cerr << "full inertia compensation\n";
      break;
    case tut04::JTask::COMPENSATION_DIAGONAL:
      std::cerr << "diagonal-only inertia compensation\n";
      break;
    default:
      std::cerr << "inertia compensation is off\n";
      break;
    }
    jspace::pretty_print(jtask->goalpos_, std::cerr, "  goalpos", "    ");
    jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
    jspace::pretty_print(jtask->goalvel_, std::cerr, "  goalvel", "    ");
    jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");
    jspace::pretty_print(command, std::cerr, "  command", "    ");
  }
  ++iteration;
  
  return true;
}


static void draw_cb(double x0, double y0, double scale)
{
  if (0 != jtask->goalpos_.rows()) {
    tutsim::draw_robot(jtask->goalpos_, 1, 100, 255, 100, x0, y0, scale);
    tutsim::draw_delta_jpos(jtask->goalpos_, 1, 100, 255, 255, x0, y0, scale);
  }
}


int main(int argc, char ** argv)
{
  try {
    model.reset(jspace::test::parse_sai_xml_file(model_filename, true));
    jtask.reset(new tut04::JTask());
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_draw_cb(draw_cb);
  return tutsim::run(servo_cb);
}
