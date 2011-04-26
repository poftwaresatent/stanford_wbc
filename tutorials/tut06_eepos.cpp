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
#include <opspace/skill_library.hpp>
#include <opspace/ClassicTaskPostureController.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <FL/fl_draw.H>
#include <err.h>


static std::string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<opspace::ClassicTaskPostureController> controller;
static boost::shared_ptr<opspace::GenericSkill> skill;
static boost::shared_ptr<opspace::CartPosTask> eetask;
static boost::shared_ptr<opspace::JPosTask> jtask;
static opspace::Parameter * eegoalpos;
static opspace::Parameter * eegoalvel;
static opspace::Parameter * jgoalpos;
static opspace::Parameter * jgoalvel;
static size_t mode;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  mode = toggle_count % 5;
  static size_t prevmode(42);
  static size_t iteration(0);
  
  jspace::Vector jpos(state.position_.rows());
  jspace::Vector jvel(state.velocity_.rows());
  for (int ii(0); ii < state.position_.rows(); ++ii) {
    static double const amplitude(0.5 * M_PI);
    double const omega(1.0 + 0.1 * ii);
    double const phase(omega * 1e-3 * wall_time_ms);
    jpos[ii] =         amplitude * sin(phase);
    jvel[ii] = omega * amplitude * cos(phase);
  }
  
  if (0 == mode) {
    
    //////////////////////////////////////////////////
    // Re-initialize simulator
    
    state.position_ = jpos;
    state.velocity_ = jvel;
    
    prevmode = mode;
    return false;
    
  }
  
  //////////////////////////////////////////////////
  // Update the model to reflect the current robot state.
  
  model->update(state);
  
  if (1 != prevmode) {
    jspace::Status st(skill->init(*model));
    if ( ! st) {
      errx(EXIT_FAILURE, "skill->init() failed: %s", st.errstr.c_str());
    }
    st = controller->init(*model);
    if ( ! st) {
      errx(EXIT_FAILURE, "controller->init() failed: %s", st.errstr.c_str());
    }
  }
  
  if ((3 == mode) || (4 == mode)) {
    static jspace::Vector pos(3), vel(3);
    static double const oy(0.2);
    static double const oz(0.37);
    static double const amp(2.5);
    double const py(oy * 1e-3 * wall_time_ms);
    double const pz(oz * 1e-3 * wall_time_ms);
    pos << 0.0,	     amp * sin(py),      amp * sin(pz);
    vel << 0.0,	oy * amp * cos(py), oz * amp * cos(pz);
    if ( ! eegoalpos->set(pos)) {
      errx(EXIT_FAILURE, "failed to set end-effector goal position");
    }
    if ( ! eegoalvel->set(vel)) {
      errx(EXIT_FAILURE, "failed to set end-effector goal velocity");
    }
  }
  
  if ((2 == mode) || (4 == mode)) {
    if ( ! jgoalpos->set(jpos)) {
      errx(EXIT_FAILURE, "failed to set joint goal position");
    }
    if ( ! jgoalvel->set(jvel)) {
      errx(EXIT_FAILURE, "failed to set joint goal velocity");
    }
  }
  
  if ( ! skill->update(*model)) {
    errx(EXIT_FAILURE, "skill update failed");
  }
  
  if ( ! controller->computeCommand(*model, *skill, command)) {
    errx(EXIT_FAILURE, "controller update failed");
  }
  
  if (0 == (iteration % 100)) {
    controller->dbg(std::cerr, "**************************************************", "");
  }
  
  ++iteration;
  prevmode = mode;
  
  return true;
}


static void draw_cb(double x0, double y0, double scale)
{
  if (0 != mode) {
    
    tutsim::draw_delta_jpos(*jgoalpos->getVector(), 1, 100, 80, 80, x0, y0, scale);
    
    //////////////////////////////////////////////////
    // Remember: we plot the YZ plane, X is sticking out of the screen
    // but the robot is planar anyway.
    
    fl_color(255, 100, 100);
    fl_line_style(FL_SOLID, 1, 0);
    
    double const gx(eegoalpos->getVector()->y());
    double const gy(eegoalpos->getVector()->z());
    int const rr(ceil(0.15 * scale));
    int const dd(2 * rr);
    fl_arc(int(x0 + gx * scale) - rr, int(y0 - gy * scale) - rr, dd, dd, 0.0, 360.0);
    
    double const vx(eegoalvel->getVector()->y());
    double const vy(eegoalvel->getVector()->z());
    double const px(gx + vx * 0.2);
    double const py(gy + vy * 0.2);
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
    
    eetask.reset(new opspace::CartPosTask("tut06-eepos"));
    jspace::Vector kp(1), kd(1), maxvel(1), ctrlpt(3);
    kp << 400.0;
    kd << 40.0;
    maxvel << 1.0;
    ctrlpt << 0.0, 0.0, -1.0;
    eetask->quickSetup(kp, kd, maxvel, "link4", ctrlpt);
    eegoalpos = eetask->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalpos) {
      errx(EXIT_FAILURE, "failed to find appropriate end-effector goalpos parameter");
    }
    eegoalvel = eetask->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalvel) {
      errx(EXIT_FAILURE, "failed to find appropriate end-effector goalvel parameter");
    }
    
    jtask.reset(new opspace::JPosTask("tut06-jtask"));
    kp << 100.0;
    kd << 20.0;
    maxvel << M_PI;
    jtask->quickSetup(kp, kd, maxvel);
    jgoalpos = jtask->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jgoalpos) {
      errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalpos parameter");
    }
    jgoalvel = jtask->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jgoalvel) {
      errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalvel parameter");
    }

    skill.reset(new opspace::GenericSkill("tut06-skill"));
    skill->appendTask(eetask);
    skill->appendTask(jtask);

    controller.reset(new opspace::ClassicTaskPostureController("tut06-ctrl"));
    
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_draw_cb(draw_cb);
  return tutsim::run(servo_cb);
}
