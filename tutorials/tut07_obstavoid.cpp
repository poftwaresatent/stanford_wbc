/*
 * Copyright (C) 2013 Roland Philippsen. All rights reserved.
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
   \file tut07_obstavoid.cpp
   \author Roland Philippsen
*/

#include "tutsim.hpp"
#include <opspace/task_library.hpp>
#include <opspace/skill_library.hpp>
#include <opspace/Factory.hpp>
#include "uta_opspace/ControllerNG.hpp"
#include "ObstAvoidTask.hpp"
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <FL/fl_draw.H>
#include <err.h>

// Workaround for Fedora 16 (and maybe others) where /usr/X11/Xlib.h
// defines Status to be int.  Beats me why they don't use a typedef,
// and I also don't get why other systems such as OS X, Debian, and
// Ubuntu do not need this workaround in spite of having the same
// preprocessor define in Xlib.h --- maybe FLTK somehow undefines it
// on those?
#undef Status

using namespace std;
using namespace jspace;


static bool verbose(false);
static boost::shared_ptr<opspace::Factory> factory;
static string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<Model> model;
static boost::shared_ptr<uta_opspace::ControllerNG> controller;
static boost::shared_ptr<opspace::Skill> skill;
static opspace::Parameter * jgoalpos;
static opspace::Parameter * jgoalvel;
static vector<opspace::Parameter *> global_obstacle;
static vector<opspace::Parameter *> global_delta;
static vector<opspace::Parameter *> global_control_point;
static vector<opspace::Parameter *> activation;
static size_t mode;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     State & state,
		     Vector & command)
{
  mode = toggle_count % 3;
  static size_t prevmode(42);
  static size_t iteration(0);
  
  Vector jpos(state.position_.rows());
  Vector jvel(state.velocity_.rows());
  for (int ii(0); ii < state.position_.rows(); ++ii) {
    static double const amplitude(0.5 * M_PI);
    double const omega(1.0 + 0.1 * ii);
    double const phase(omega * 1e-3 * wall_time_ms);
    jpos[ii] =         amplitude * sin(phase);
    jvel[ii] = omega * amplitude * cos(phase);
  }
  
  {
    static Vector pos(3);
    static double const oy(0.2);
    static double const oz(0.37);
    static double const amp(2.5);
    double const py(oy * 1e-3 * wall_time_ms);
    double const pz(oz * 1e-3 * wall_time_ms);
    pos << 0.0,	     amp * sin(py),      amp * sin(pz);
    if (global_obstacle.empty()) {
      errx(EXIT_FAILURE, "no global obstacle parameters to write to...");
    }
    for (size_t ii(0); ii < global_obstacle.size(); ++ii) {
      if ( ! global_obstacle[ii]->set(pos)) {
	errx(EXIT_FAILURE, "failed to set global obstacle position");
      }
    }
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
  
  if ((1 == mode) && (1 != prevmode)) {
    Status st(skill->init(*model));
    if ( ! st) {
      errx(EXIT_FAILURE, "skill->init() failed: %s", st.errstr.c_str());
    }
    st = controller->init(*model);
    if ( ! st) {
      errx(EXIT_FAILURE, "controller->init() failed: %s", st.errstr.c_str());
    }
  }
  
  if (2 == mode) {
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
    skill->dump(cerr, "**************************************************", "");
    controller->dbg(cerr, "==================================================", "");
  }
  
  ++iteration;
  prevmode = mode;
  
  return true;
}


static void draw_cb(double x0, double y0, double scale)
{
  //////////////////////////////////////////////////
  // Remember: we plot the YZ plane, X is sticking out of the screen
  // but the robot is planar anyway.
  
  if ( ! global_obstacle.empty()) {
    fl_color(255, 100, 100);
    fl_line_style(FL_SOLID, 3, 0);
    
    double const gx(global_obstacle[0]->getVector()->y());
    double const gy(global_obstacle[0]->getVector()->z());
    int const rr(ceil(0.2 * scale));
    int const dd(2 * rr);
    fl_arc(int(x0 + gx * scale) - rr, int(y0 - gy * scale) - rr, dd, dd, 0.0, 360.0);
  }
  
  if (0 != mode) {
    
    tutsim::draw_delta_jpos(*jgoalpos->getVector(), 3, 100, 80, 80, x0, y0, scale);
    
    for (size_t ii(0); ii < global_delta.size(); ++ii) {
      double const gx(global_control_point[ii]->getVector()->y());
      double const gy(global_control_point[ii]->getVector()->z());
      double const px(gx + global_delta[ii]->getVector()->y());
      double const py(gy + global_delta[ii]->getVector()->z());
      if (*(activation[ii]->getReal()) > 0.0) {
	fl_color(255, 100, 255);
	fl_line_style(FL_SOLID, 3, 0);
      }
      else {
	fl_color(100, 255, 100);
	fl_line_style(FL_SOLID, 1, 0);
      }
      fl_line(x0 + gx * scale, y0 - gy * scale,
	      x0 + px * scale, y0 - py * scale);
    }
  }
}


static void parse_options(int argc, char ** argv)
{
  string skill_spec("");
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      errx (EXIT_FAILURE, "problem with option `%s'", argv[ii]);
    }
    else
      switch (argv[ii][1]) {
      case 'v':
	verbose = true;
 	break;
      case 's':
 	++ii;
 	if (ii >= argc) {
	  errx (EXIT_FAILURE, "-s requires parameter");
 	}
	skill_spec = argv[ii];
 	break;
      default:
	errx (EXIT_FAILURE, "invalid option `%s'", argv[ii]);
      }
  }
  
  if (skill_spec.empty()) {
    errx (EXIT_FAILURE, "please specify a skill YAML file using -s <filename>");
  }
  
  opspace::Factory::addTaskType<pws::ObstAvoidTask>("pws::ObstAvoidTask");
  factory.reset(new opspace::Factory());
  Status st(factory->parseFile(skill_spec));
  if ( ! st) {
    errx(EXIT_FAILURE,
	 "parse error: %s (%s)",
	 skill_spec.c_str(), st.errstr.c_str());
  }
  if (verbose) {
    factory->dump(cout, "*** parsed tasks and skills", "* ");
  }
  
  opspace::Factory::task_table_t const & tt(factory->getTaskTable());
  opspace::Parameter * parm;
  for (opspace::Factory::task_table_t::const_iterator it(tt.begin()); it != tt.end(); ++it) {
    
    if ((*it)->getName() == "jpos") {
      jgoalpos = (*it)->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
      if ( ! jgoalpos) {
	errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalpos parameter");
      }
      jgoalvel = (*it)->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
      if ( ! jgoalvel) {
	errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalvel parameter");
      }
      
    }
    else {
      
      parm = (*it)->lookupParameter("global_obstacle", opspace::PARAMETER_TYPE_VECTOR);
      if (parm) {
	global_obstacle.push_back(parm);
      }
      parm = (*it)->lookupParameter("global_control_point", opspace::PARAMETER_TYPE_VECTOR);
      if (parm) {
	global_control_point.push_back(parm);
      }
      parm = (*it)->lookupParameter("global_delta", opspace::PARAMETER_TYPE_VECTOR);
      if (parm) {
	global_delta.push_back(parm);
      }
      parm = (*it)->lookupParameter("activation", opspace::PARAMETER_TYPE_REAL);
      if (parm) {
	activation.push_back(parm);
      }
      
    }
  }
  if ( ! jgoalpos) {
    errx(EXIT_FAILURE, "failed to find jpos task");
  }
  
  if (factory->getSkillTable().empty()) {
    errx (EXIT_FAILURE, "empty skill table");
  }
  skill = factory->getSkillTable()[0];
}


int main(int argc, char ** argv)
{
  try {
    parse_options(argc, argv);
    model.reset(test::parse_sai_xml_file(model_filename, true));
    controller.reset(new uta_opspace::ControllerNG("tut07-ctrl"));
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_draw_cb(draw_cb);
  return tutsim::run(servo_cb);
}
