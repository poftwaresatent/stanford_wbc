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
   \file tut06_eepos.cpp
   \author Roland Philippsen
   
   This tutorial shows Cartesian trajectory tracking task with a
   controller joint posture in the nullspace of the task. It shows how
   to implement this behavior with opspace::Task subclasses from the
   provided task library, and by relying on the provided
   opspace::ClassicTaskPostureController to perform the dynamically
   consistent nullspace projection. It also introduces the concept of
   opspace::Skill, which provides a container for task
   hierarchies. For this example, we simply use the
   opspace::GenericSkill.
   
   As usual, when the simulation starts, it is in
   swaying-reinitialization mode. After clicking Toggle once, the
   robot holds the current position in Cartesian as well as joint
   space.  Clicking Toggle a second time starts a swaying joint-space
   motion in the nullspace of the task: the robot will attempt to
   minimize the difference between the actual and desired joint
   angles, without moving the position of the end-effector
   point. Clicking Toggle again enters Cartesian trajectory tracking
   mode with a constant joint posture, and clicking Toggle a fourth
   time makes the posture change continously while tracking the
   trajectory.
   
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
