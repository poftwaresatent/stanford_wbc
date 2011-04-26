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
#include <opspace/task_library.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <FL/fl_draw.H>
#include <err.h>


static std::string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<opspace::CartPosTask> task;
static opspace::Parameter * goalpos;
static opspace::Parameter * goalvel;
static size_t mode;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  mode = toggle_count % 3;
  static size_t prevmode(42);
  static size_t iteration(0);
  
  if (0 == mode) {
    
    //////////////////////////////////////////////////
    // Re-initialize simulator
    
    for (int ii(0); ii < state.position_.rows(); ++ii) {
      static double const amplitude(0.5 * M_PI);
      double const omega(1.0 + 0.1 * ii);
      double const phase(omega * 1e-3 * wall_time_ms);
      state.position_[ii] =         amplitude * sin(phase);
      state.velocity_[ii] = omega * amplitude * cos(phase);
    }
    prevmode = mode;
    return false;
    
  }
  
  //////////////////////////////////////////////////
  // Update the model to reflect the current robot state.
  
  model->update(state);
  
  if (1 != prevmode) {
    jspace::Status const st(task->init(*model));
    if ( ! st) {
      errx(EXIT_FAILURE, "task->init() failed: %s", st.errstr.c_str());
    }
  }
  if (2 == mode) {
    static jspace::Vector pos(3), vel(3);
    static double const oy(0.2);
    static double const oz(0.37);
    static double const amp(2.5);
    double const py(oy * 1e-3 * wall_time_ms);
    double const pz(oz * 1e-3 * wall_time_ms);
    pos << 0.0,	     amp * sin(py),      amp * sin(pz);
    vel << 0.0,	oy * amp * cos(py), oz * amp * cos(pz);
    if ( ! goalpos->set(pos)) {
      errx(EXIT_FAILURE, "failed to set end-effector goal position");
    }
    if ( ! goalvel->set(vel)) {
      errx(EXIT_FAILURE, "failed to set end-effector goal velocity");
    }
  }
  
  task->update(*model);
  
  //////////////////////////////////////////////////
  // The end-effector position task computes a desired acceleration in
  // Cartesian space. In order to send it to the joints as desired
  // torques, this needs to be translated from operational space to
  // joint space. In the opspace library, this is the job of
  // opspace::Controller::computeCommand(). However, that needs a bit
  // of extra infrastructure, so here we simply hardcode the
  // fundamental equation without any frosting...
  
  jspace::Matrix aa;
  if ( ! model->getMassInertia(aa)) {
    errx(EXIT_FAILURE, "model->getMassInertia() failed");
  }
  jspace::Vector gg;
  if ( ! model->getGravity(gg)) {
    errx(EXIT_FAILURE, "model->getGravity() failed");
  }
  command = aa * task->getJacobian().transpose() * task->getCommand() + gg;
  
  ++iteration;
  prevmode = mode;
  
  return true;
}


static void draw_cb(double x0, double y0, double scale)
{
  if (0 != mode) {
    
    //////////////////////////////////////////////////
    // Remember: we plot the YZ plane, X is sticking out of the screen
    // but the robot is planar anyway.
    
    fl_color(255, 100, 100);
    fl_line_style(FL_SOLID, 1, 0);
    
    double const gx(goalpos->getVector()->y());
    double const gy(goalpos->getVector()->z());
    int const rr(ceil(0.15 * scale));
    int const dd(2 * rr);
    fl_arc(int(x0 + gx * scale) - rr, int(y0 - gy * scale) - rr, dd, dd, 0.0, 360.0);
    
    double const vx(goalvel->getVector()->y());
    double const vy(goalvel->getVector()->z());
    double const px(gx + vx * 0.1);
    double const py(gy + vy * 0.1);
    fl_line(x0 + (gx + 0.2) * scale, y0 - gy * scale,
	    x0 + (gx - 0.2) * scale, y0 - gy * scale);
    fl_line(x0 + gx * scale, y0 - (gy + 0.2) * scale,
	    x0 + gx * scale, y0 - (gy - 0.2) * scale);
    fl_color(255, 255, 100);
    fl_line(x0 + gx * scale, y0 - gy * scale,
	    x0 + px * scale, y0 - py * scale);
  }
}


int main(int argc, char ** argv)
{
  try {
    model.reset(jspace::test::parse_sai_xml_file(model_filename, true));
    task.reset(new opspace::CartPosTask("tut05"));
    jspace::Vector kp(1), kd(1), maxvel(1), ctrlpt(3);
    kp << 400.0;
    kd << 40.0;
    maxvel << 1.0;
    ctrlpt << 0.0, 0.0, -1.0;
    task->quickSetup(kp, kd, maxvel, "link4", ctrlpt);
    
    //////////////////////////////////////////////////
    // Here's an example of how to use the parameter reflection aspect
    // of the opspace library. We get a pointer to the goalpos and
    // goalvel parameters, which allows us direct read/write access to
    // them, e.g. for plotting of the end-effector goal position. Here
    // we know to expect a vector parameter, so we use the more
    // stringent version of lookupParameter().
    
    goalpos = task->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! goalpos) {
      errx(EXIT_FAILURE, "failed to find appropriate goalpos parameter");
    }
    goalvel = task->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! goalvel) {
      errx(EXIT_FAILURE, "failed to find appropriate goalvel parameter");
    }
    
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_draw_cb(draw_cb);
  return tutsim::run(servo_cb);
}
