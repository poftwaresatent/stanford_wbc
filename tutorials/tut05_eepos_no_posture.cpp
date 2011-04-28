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
   \file tut05_opspace_and_parameters.cpp
   \author Roland Philippsen
   
   This tutorial, shows how to create an operational space task with
   reflected parameters. The task space is the plane, and the two
   parameters are the planar position and velocity of the
   end-effector, and the task-space servo is implemented in
   tut05::PTask by subclassing opspace::Task. The two parameters that
   get reflected are tut05::PTask::goalpos_ and
   tut05::PTask::goalvel_, and the reflection is achieved with calls
   to opspace::ParameterReflection::declareParameter() within the
   constructor of tut05::PTask.
   
   When the simulation starts, the robot follows the usual swaying
   motion by re-initializing the simulation at each tick. After
   pressing Toggle, the planar task is used to track a goal point
   which is continuously moving around the plane. Initially, the raw
   J^T*F_op torque is used, without any compensation for gravity or
   dynamic effects, which leads to very poor tracking
   performance. Clicking Toggle a second time switches on gravity
   compensation, which makes the robot sag less. After a third click
   on Toggle, full inertial and Coriolis compensation is enabled,
   which leads to better performance. After pressing Toggle again, the
   cycle continues.
   
   Note that the planar operational task has two dimensions, and given
   that the simulated robot has four degrees of freedom, this means
   that two degrees of redundancy remain. This is evidenced by the
   behavior of the robot, which keeps swinging around in the nullspace
   of the task. This undetermined nullspace motion leads to undesired
   effects on the tracking of the planar goal. In order to get rid of
   these disturbances, we need to add a well-defined control in the
   task nullspace, which is done in the tut06_eepos.cpp tutorial.
   
*/

#include "tutsim.hpp"
#include <opspace/task_library.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <FL/fl_draw.H>
#include <err.h>


namespace tut05 {
  
  using namespace jspace;

  class PTask : public opspace::Task {
  public:
    PTask()
      : opspace::Task("tut05::PTask"),
	initialized_(false)
    {
      declareParameter("goalpos", &goalpos_);
      declareParameter("goalvel", &goalvel_);
    }
    
    void updateStateAndJacobian(Model const & model)
    {
      jspace::Transform ee_transform;
      model.computeGlobalFrame(model.getNode(3), // hardcoded end-effector ID = 3
			       0, 0, -1, // hardcoded ctrl point 1m down along Z axis
			       ee_transform);
      actual_ = ee_transform.translation().block(1, 0, 2, 1); // extract 2D point (y, z)
      Matrix Jfull;
      model.computeJacobian(model.getNode(3),
			    ee_transform.translation(),
			    Jfull);
      jacobian_ = Jfull.block(1, 0, 2, Jfull.cols()); // extract the Y and Z rows
      curvel_ = jacobian_ * model.getState().velocity_;
    }
    
    virtual jspace::Status init(jspace::Model const & model)
    {
      //////////////////////////////////////////////////
      // Initialize our PD parameters.
      
      kp_ = 32.0;
      kd_ = 8.0;
      
      //////////////////////////////////////////////////
      // The Jacobian becomes important now, because we are going to
      // use it to compute the mapping from operational space force to
      // joint torques. For real applications, it would furthermore be
      // used to determine the nullspace of the operational point
      // motion (but not in this tutorial where we simply let the
      // robot "flop around" in the task nullspace).
      
      updateStateAndJacobian(model);
      
      //////////////////////////////////////////////////
      // Initialize our goal to the current state.
      
      goalpos_ = actual_;
      goalvel_ = jspace::Vector::Zero(2);
      
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
      // Update the state and Jacobian of our task.
      
      updateStateAndJacobian(model);
      
      //////////////////////////////////////////////////
      // Compute PD control forces (in task space). Later, in the
      // control loop, our Jacobian will be used to map them to
      // joint-space.  Inertial and gravity compensation happens
      // there, too. In effect, an operational space task only worries
      // about desired accelerations within its own task space.
      
      command_ = kp_ * (goalpos_ - actual_) + kd_ * (goalvel_ - curvel_);
      
      jspace::Status ok;
      return ok;
    }

    bool initialized_;    
    double kp_, kd_;
    jspace::Vector curvel_;
    jspace::Vector goalpos_, goalvel_;
    
    // Make these two protected fields publicly available for easier
    // info messages.
    opspace::Task::actual_;
    opspace::Task::jacobian_;
  };
  
}


static std::string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<jspace::Model> model;
static tut05::PTask ptask;
static opspace::Parameter * goalpos;
static opspace::Parameter * goalvel;
static size_t mode;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  mode = toggle_count % 4;
  static size_t prevmode(42);
  
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
  
  if (0 == prevmode) {
    jspace::Status const st(ptask.init(*model));
    if ( ! st) {
      errx(EXIT_FAILURE, "ptask.init() failed: %s", st.errstr.c_str());
    }
  }
  
  static jspace::Vector pos(2), vel(2);
  static double const ox(0.2);
  static double const oy(0.37);
  static double const amp(2.5);
  double const px(ox * 1e-3 * wall_time_ms);
  double const py(oy * 1e-3 * wall_time_ms);
  pos <<      amp * sin(px),      amp * sin(py);
  vel << ox * amp * cos(px), oy * amp * cos(py);
  if ( ! goalpos->set(pos)) {
    errx(EXIT_FAILURE, "failed to set end-effector goal position");
  }
  if ( ! goalvel->set(vel)) {
    errx(EXIT_FAILURE, "failed to set end-effector goal velocity");
  }
  
  ptask.update(*model);
  
  //////////////////////////////////////////////////
  // The end-effector position task computes a desired acceleration in
  // Cartesian space. In order to send it to the joints as desired
  // torques, this needs to be translated from operational space to
  // joint space. In the opspace library, this is the job of
  // opspace::Controller::computeCommand(). However, that needs a bit
  // of extra infrastructure, so here we simply hardcode the
  // fundamental equation without any frosting...
  
  jspace::Matrix aa;
  model->getMassInertia(aa);
  jspace::Vector gg;
  model->getGravity(gg);
  jspace::Vector cc;
  model->getCoriolisCentrifugal(cc);
  
  char const * mode_desc;
  switch (mode) {
  case 1:
    mode_desc = "no compensation\n";
    command = ptask.getJacobian().transpose() * ptask.getCommand();
    break;
  case 2:
    mode_desc = "only gravity compensation\n";
    command = ptask.getJacobian().transpose() * ptask.getCommand() + gg;
    break;
  default:
    mode_desc = "full compensation\n";
    command = aa * ptask.getJacobian().transpose() * ptask.getCommand() + gg + cc;
  }
  
  static size_t iteration(0);
  if (0 == (iteration % 100)) {
    std::cerr << "**************************************************\n"
	      << mode_desc;
    jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
    jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");
    jspace::pretty_print(ptask.actual_, std::cerr, "  ppos", "    ");
    jspace::pretty_print(ptask.curvel_, std::cerr, "  pvel", "    ");
    jspace::pretty_print(ptask.goalpos_, std::cerr, "  goalpos", "    ");
    jspace::pretty_print(ptask.goalvel_, std::cerr, "  goalvel", "    ");
    jspace::pretty_print(ptask.jacobian_, std::cerr, "  Jacobian", "    ");
    jspace::pretty_print(command, std::cerr, "  command", "    ");
  }
  ++iteration;
  
  prevmode = mode;
  return true;
}


static void draw_cb(double x0, double y0, double scale)
{
  if (0 != mode) {
    
    //////////////////////////////////////////////////
    // On a side-note: we plot the YZ plane, X is sticking out of the
    // screen (but the robot is planar anyway). However, the
    // tut05::PTask's planar operational space already takes that into
    // account.
    
    fl_color(255, 100, 100);
    fl_line_style(FL_SOLID, 1, 0);
    
    double const gx(goalpos->getVector()->x());
    double const gy(goalpos->getVector()->y());
    int const rr(ceil(0.15 * scale));
    int const dd(2 * rr);
    fl_arc(int(x0 + gx * scale) - rr, int(y0 - gy * scale) - rr, dd, dd, 0.0, 360.0);
    
    double const vx(goalvel->getVector()->x());
    double const vy(goalvel->getVector()->y());
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
    goalpos = ptask.lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! goalpos) {
      errx(EXIT_FAILURE, "failed to find appropriate goalpos parameter");
    }
    goalvel = ptask.lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
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
