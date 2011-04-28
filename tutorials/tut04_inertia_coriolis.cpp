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
   \file tut04_inertia_coriolis.cpp
   \author Roland Philippsen
   
   Demonstration of inertial decoupling and Coriolis/centrifugal
   compensation. This tutorial uses a joint-space task which always
   uses gravity compensation, but the terms for inertial decoupling
   and Coriolis/centrifugal compensation can be switched on and off at
   runtime. When you start the demo, it tries to track a joint-space
   trajectory using a PD controller. When you hit Toggle, it switches
   on the inertia decoupling (desired torques get pre-multiplied by
   the joint-space mass-inertia matrix). After clicking Toggle a
   second time, it also adds the predicted Coriolis/centrifugal
   terms. As usual, after that the Toggle cycle repeats.
   
*/

#include "tutsim.hpp"
#include <opspace/Task.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <err.h>


namespace tut04 {
  
  class JTask : public opspace::Task {
  public:
    JTask()
      : opspace::Task("tut04::JTask"),
	initialized_(false),
	inertia_compensation_(true),
	coriolis_compensation_(true)
    {
    }
    
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
      
      if (inertia_compensation_) {
	jspace::Matrix aa;
	if ( ! model.getMassInertia(aa)) {
	  return jspace::Status(false, "failed to retrieve inertia");
	}
	command_ = aa * gamma;
      }
      else {
	command_ = gamma;
      }
      
      if (coriolis_compensation_) {
	jspace::Vector cc;
	if ( ! model.getCoriolisCentrifugal(cc)) {
	  return jspace::Status(false, "failed to retrieve coriolis");
	}
	command_ += cc;
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
    bool inertia_compensation_;
    bool coriolis_compensation_;
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
  //////////////////////////////////////////////////
  // Update the model to reflect the current robot state.
  
  model->update(state);

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
  
  //////////////////////////////////////////////////
  // Cycle through inertia/coriolis compensation modes.
  
  switch (toggle_count % 3) {
  case 0:
    jtask->inertia_compensation_ = false;
    jtask->coriolis_compensation_ = false;
    break;
  case 1:
    jtask->inertia_compensation_ = true;
    jtask->coriolis_compensation_ = false;
    break;
  case 2:
  default:
    jtask->inertia_compensation_ = true;
    jtask->coriolis_compensation_ = true;
  }
  
  //////////////////////////////////////////////////
  // Compute the command using jtask... as usual.
  
  jtask->update(*model);
  command = jtask->getCommand();
  
  //////////////////////////////////////////////////
  // Print debug info from time to time.
  
  static size_t iteration(0);
  if (0 == (iteration % 100)) {
    if (jtask->inertia_compensation_) {
      std::cerr << "inertia compensation is ON\n";
    }
    else {
      std::cerr << "inertia compensation is off\n";
    }
    if (jtask->coriolis_compensation_) {
      std::cerr << "coriolis compensation is ON\n";
      jspace::Vector cc;
      model->getCoriolisCentrifugal(cc);
      jspace::pretty_print(cc, std::cerr, "  Coriolis/centrifugal torques", "    ");
    }
    else {
      std::cerr << "coriolis compensation is off\n";
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
