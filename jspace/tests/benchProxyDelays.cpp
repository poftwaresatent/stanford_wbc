/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file benchProxyDelays.cpp
   \author Roland Philippsen
*/

#include <jspace/test/model_library.hpp>
#include <jspace/State.hpp>
#include <jspace/Model.hpp>
#include <jspace/RobotAPI.hpp>
#include <jspace/Controller.hpp>
#include <jspace/robot_proxy.hpp>
#include <jspace/proxy_util.hpp>
#include <wbcnet/misc/DelayHistogram.hpp>
#include <wbcnet/imp/RawMemChannel.hpp>
#include <err.h>
#include <stdlib.h>
#include <stdio.h>


namespace {
  
  struct custom_ds_t {
    double pos[6];
    double vel[6];
    double tau[6];
    int tick;
    char hello[64];
    double eepos[3];
  };
  
  static void custom_ds_to_state(custom_ds_t const * ds, jspace::State & state);
  static void custom_controller(jspace::Model const & model, custom_ds_t * ds);
  
  
  class CustomRobot
    : public jspace::RobotAPI
  {
  public:
    explicit CustomRobot(custom_ds_t * ds);
    
    virtual jspace::Status readState(jspace::State & state);
    virtual jspace::Status writeCommand(std::vector<double> const & command);
    virtual void shutdown();
    
  protected:
    custom_ds_t * ds_;
  };
  
  
  class CustomController
    : public jspace::Controller
  {
  public:
    explicit CustomController(custom_ds_t * ds);
    
    virtual jspace::Status setGoal(std::vector<double> const & goal);
    virtual jspace::Status getGoal(std::vector<double> & goal) const;
    virtual jspace::Status getActual(std::vector<double> & actual) const;
    
    virtual jspace::Status setGains(std::vector<double> const & kp, std::vector<double> const & kd);
    virtual jspace::Status getGains(std::vector<double> & kp, std::vector<double> & kd) const;
    
    virtual jspace::Status latch(jspace::Model const & model);
    virtual jspace::Status computeCommand(jspace::Model const & model, std::vector<double> & tau);
    
  protected:
    custom_ds_t * ds_;
  };
    
}


static void bench_raw(jspace::Model & model, jspace::State & state, custom_ds_t * ds,
		      int nticks, wbcnet::DelayHistogram & dhist);
static void bench_wrapped(jspace::Model & model, jspace::State & state, custom_ds_t * ds,
			  int nticks, wbcnet::DelayHistogram & dhist);
static void bench_proxified(jspace::Model & model, jspace::State & state, custom_ds_t * ds,
			    int nticks, wbcnet::DelayHistogram & dhist);
static void fill_ds(int tick, custom_ds_t * ds);
static void dump_ds(custom_ds_t const * ds);

int main(int argc, char ** argv)
{
  try {
    jspace::Model * puma_model(jspace::test::create_puma_model());
    jspace::State puma_state(6, 6, 0);
    custom_ds_t ds;
    
    wbcnet::DelayHistogram * dhist[3];
    for (int ii(0); ii < 3; ++ii) {
      dhist[ii] = new wbcnet::DelayHistogram(5, 5, 0, 2);
      if ( ! (dhist[ii]->SetName(0, "ds to state")
	      && dhist[ii]->SetName(1, "model update")
	      && dhist[ii]->SetName(2, "controller")
	      && dhist[ii]->SetName(3, "command to ds")
	      && dhist[ii]->SetName(4, "TOTAL"))) {
	errx(EXIT_FAILURE, "dhist->SetName() oops");
      }
    }
    
    bench_raw(*puma_model, puma_state, &ds, 500, *dhist[0]);
    bench_wrapped(*puma_model, puma_state, &ds, 500, *dhist[1]);
    bench_proxified(*puma_model, puma_state, &ds, 500, *dhist[2]);
    
    printf("\nDelay and computation time measurements:\n"
	   "raw:\n");
    if ( ! dhist[0]->DumpTable(stdout)) {
      printf("oops, could not print\n");
    }

    printf("wrapped:\n");
    if ( ! dhist[1]->DumpTable(stdout)) {
      printf("oops, could not print\n");
    }

    printf("proxified:\n");
    if ( ! dhist[2]->DumpTable(stdout)) {
      printf("oops, could not print\n");
    }
    
    printf("percent of time spent in the various steps of the servo iteration:\n"
	   "           | shuffling |     model |   control |\n"
	   "-----------+-----------+-----------+-----------+\n"
	   "raw:       |    % 6.2f |    % 6.2f |    % 6.2f |\n"
	   "wrapped:   |    % 6.2f |    % 6.2f |    % 6.2f |\n"
	   "proxified: |    % 6.2f |    % 6.2f |    % 6.2f |\n",
	   100 * (dhist[0]->GetMsMean(0) + dhist[0]->GetMsMean(3)) / dhist[0]->GetMsMean(4),
	   100 * dhist[0]->GetMsMean(1) / dhist[0]->GetMsMean(4),
	   100 * dhist[0]->GetMsMean(2) / dhist[0]->GetMsMean(4),
	   100 * (dhist[1]->GetMsMean(0) + dhist[1]->GetMsMean(3)) / dhist[1]->GetMsMean(4),
	   100 * dhist[1]->GetMsMean(1) / dhist[1]->GetMsMean(4),
	   100 * dhist[1]->GetMsMean(2) / dhist[1]->GetMsMean(4),
	   100 * (dhist[2]->GetMsMean(0) + dhist[2]->GetMsMean(3)) / dhist[2]->GetMsMean(4),
	   100 * dhist[2]->GetMsMean(1) / dhist[2]->GetMsMean(4),
	   100 * dhist[2]->GetMsMean(2) / dhist[2]->GetMsMean(4));
    
    // Don't forget to clean up after ourselves
    delete puma_model;
    for (int ii(0); ii < 3; ++ii) {
      delete dhist[ii];
    }
    
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
}


void bench_raw(jspace::Model & model, jspace::State & state, custom_ds_t * ds,
	       int nticks, wbcnet::DelayHistogram & dhist)
{
  struct timeval t0, t1, t2, t4; // there is no t3, it's the same as t2
  
  for (int tick(0); tick < nticks; ++tick) {
    fill_ds(tick, ds);
    
    // Update the model according to our state
    if (0 != gettimeofday(&t0, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    custom_ds_to_state(ds, state);
    if (0 != gettimeofday(&t1, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    model.update(state);
    
    // Update controller... this could of course just as easily call
    // something from jspace/controller_library, in which case we'd
    // have to convert from std::vector<double> to double[6], which
    // is a trivial memcpy.
    if (0 != gettimeofday(&t2, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    custom_controller(model, ds);
    
    if (0 != gettimeofday(&t4, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    
    if ( ! (dhist.StartStop(   0, &t0, &t1)
	    && dhist.StartStop(1, &t1, &t2)
	    && dhist.StartStop(2, &t2, &t4)
	    && dhist.StartStop(3, &t2, &t2) // yes, use the same t2 twice because there is no cmd->ds
	    && dhist.StartStop(4, &t0, &t4))) {
      errx(EXIT_FAILURE, "dhist.StartStop() oops");
    }
    
    dump_ds(ds);
  }
}


void bench_wrapped(jspace::Model & model, jspace::State & state, custom_ds_t * ds,
		   int nticks, wbcnet::DelayHistogram & dhist)
{
  struct timeval t0, t1, t2, t3, t4;
  CustomRobot robot(ds);
  CustomController controller(ds);
  jspace::Status status;
  std::vector<double> tau;
  
  for (int tick(0); tick < nticks; ++tick) {
    fill_ds(tick, ds);
    
    // Update the model according to our state
    if (0 != gettimeofday(&t0, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    status = robot.readState(state);
    if ( ! status) {
      errx(EXIT_FAILURE, "robot.readState(): %s", status.errstr.c_str());
    }
    if (0 != gettimeofday(&t1, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    model.update(state);
    if (0 != gettimeofday(&t2, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    status = controller.computeCommand(model, tau);
    if ( ! status) {
      errx(EXIT_FAILURE, "controller.computeCommand(): %s", status.errstr.c_str());
    }
    if (0 != gettimeofday(&t3, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    status = robot.writeCommand(tau);
    if ( ! status) {
      errx(EXIT_FAILURE, "robot.writeCommand(): %s", status.errstr.c_str());
    }
    if (0 != gettimeofday(&t4, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    
    if ( ! (dhist.StartStop(   0, &t0, &t1)
	    && dhist.StartStop(1, &t1, &t2)
	    && dhist.StartStop(2, &t2, &t3)
	    && dhist.StartStop(3, &t3, &t4)
	    && dhist.StartStop(4, &t0, &t4))) {
      errx(EXIT_FAILURE, "dhist.StartStop() oops");
    }
    
    dump_ds(ds);
  }
}


void bench_proxified(jspace::Model & model, jspace::State & state, custom_ds_t * ds,
		     int nticks, wbcnet::DelayHistogram & dhist)
{
  struct timeval t0, t1, t2, t3, t4;
  CustomRobot custom_robot(ds);
  CustomController controller(ds);
  jspace::Status status;
  std::vector<double> tau;
  
  static int const bufsize(512);
  std::vector<char> buffer(bufsize);
  wbcnet::RawMemChannel channel(&buffer[0], bufsize, false);
  
  // Pretend that the robot "lives" in another thread, and that all we
  // have access to here is a proxy. For that purpose, the other
  // thread needs to provide a robot server, and in this thread we
  // need a robot client. Given that we are not actually
  // multi-threaded in this benchmark, all these things live right
  // here though.
  
  jspace::RobotProxyServer robot_server;
  status = robot_server.init(&custom_robot, false, &channel, false);
  if ( ! status) {
    errx(EXIT_FAILURE, "robot_server.init(): %s", status.errstr.c_str());
  }
  
  jspace::RobotProxyClient robot_client;
  status = robot_client.init(&channel, false, jspace::CreateSPTransactionPolicy(&robot_server), true);
  if ( ! status) {
    errx(EXIT_FAILURE, "robot_client.init(): %s", status.errstr.c_str());
  }
  
  for (int tick(0); tick < nticks; ++tick) {
    fill_ds(tick, ds);
    
    // Update the model according to our state
    if (0 != gettimeofday(&t0, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    status = robot_client.readState(state);
    if ( ! status) {
      errx(EXIT_FAILURE, "robot.readState(): %s", status.errstr.c_str());
    }
    if (0 != gettimeofday(&t1, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    model.update(state);
    if (0 != gettimeofday(&t2, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    status = controller.computeCommand(model, tau);
    if ( ! status) {
      errx(EXIT_FAILURE, "controller.computeCommand(): %s", status.errstr.c_str());
    }
    if (0 != gettimeofday(&t3, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    status = robot_client.writeCommand(tau);
    if ( ! status) {
      errx(EXIT_FAILURE, "robot.writeCommand(): %s", status.errstr.c_str());
    }
    if (0 != gettimeofday(&t4, 0)) {
      err(EXIT_FAILURE, "gettimeofday");
    }
    
    if ( ! (dhist.StartStop(   0, &t0, &t1)
	    && dhist.StartStop(1, &t1, &t2)
	    && dhist.StartStop(2, &t2, &t3)
	    && dhist.StartStop(3, &t3, &t4)
	    && dhist.StartStop(4, &t0, &t4))) {
      errx(EXIT_FAILURE, "dhist.StartStop() oops");
    }
    
    dump_ds(ds);
  }
}


void fill_ds(int tick, custom_ds_t * ds)
{
  // Pretend the DS is somehow hooked up to a simulator or whatever...
  for (int ii(0); ii < 6; ++ii) {
    ds->pos[ii] = 0.1 * (tick + ii);
    ds->vel[ii] = 0.05 * (tick - ii);
  }
  ds->tick = tick;
  if (tick % 2) {
    strncpy(ds->hello, "odd", 63);
  }
  else {
    strncpy(ds->hello, "even", 63);
  }
}


void dump_ds(custom_ds_t const * ds)
{
  printf("tick: %d (%s)\n"
	 "  pos = %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n"
	 "  vel = %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n"
	 "  tau = %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n"
	 "  eepos = %5.2f %5.2f %5.2f\n",
	 ds->tick, ds->hello,
	 ds->pos[0], ds->pos[1], ds->pos[2], ds->pos[3], ds->pos[4], ds->pos[5], 
	 ds->vel[0], ds->vel[1], ds->vel[2], ds->vel[3], ds->vel[4], ds->vel[5], 
	 ds->tau[0], ds->tau[1], ds->tau[2], ds->tau[3], ds->tau[4], ds->tau[5],
	 ds->eepos[0], ds->eepos[1], ds->eepos[2]);
}


namespace {
  
  void custom_ds_to_state(custom_ds_t const * ds, jspace::State & state)
  {
    // Redundant, but in a dynamically resizing scenario we'd have to do
    // this, so let's see what sort of overhead it incurs
    state.position_.resize(6);
    state.velocity_.resize(6);
  
    memcpy(&state.position_[0], ds->pos, 6 * sizeof(double));
    memcpy(&state.velocity_[0], ds->pos, 6 * sizeof(double));
  }


  void custom_controller(jspace::Model const & model, custom_ds_t * ds)
  {
    // Get interesting stuff from the model.
    taoDNode * ee(model.getNode(5));
    if ( ! ee) {
      errx(EXIT_FAILURE, "no end effector");
    }
    jspace::Transform eeframe;
    if ( ! model.getGlobalFrame(ee, eeframe)) {
      errx(EXIT_FAILURE, "no end effector frame");
    }
    jspace::Matrix J;
    if ( ! model.computeJacobian(ee, J)) {
      errx(EXIT_FAILURE, "no end effector Jacobian");
    }
    jspace::Matrix A;
    if ( ! model.getMassInertia(A)) {
      errx(EXIT_FAILURE, "no mass inertia");
    }
  
    // Compute torques.
    jspace::Vector delta(eeframe.translation());
    double const dist(delta.norm());
    delta /= -dist;
    double strength(dist * dist * 10);
    if (strength > 1) {
      strength = 1;
    }
    jspace::Vector F(6);
    F[0] = delta[0] * strength;
    F[1] = delta[1] * strength;
    F[2] = delta[2] * strength;
    jspace::Vector tmp, tau;	// rfct from saimatrix to eigen2, can probably remove tmp
    tmp = J.transpose() * F;
    tau = A * tmp;
    
    // Store the result in our custom data structure.
    memcpy(ds->tau, tau.data(), 6 * sizeof(double));
    
    // Store the end-effector position as additional controller output
    // in the custom data structure.
    memcpy(ds->eepos, eeframe.translation().data(), 3 * sizeof(double));
  }
  
  
  CustomRobot::
  CustomRobot(custom_ds_t * ds)
    : ds_(ds)
  {
  }
  
  
  jspace::Status CustomRobot::
  readState(jspace::State & state)
  {
    custom_ds_to_state(ds_, state);
    jspace::Status ok;
    return ok;
  }
  
  
  jspace::Status CustomRobot::
  writeCommand(std::vector<double> const & command)
  {
    jspace::Status status;
    if (command.size() != 6) {
      status.ok = false;
      status.errstr = "wrong dimension";
      return status;
    }
    memcpy(ds_->tau, &command[0], 6 * sizeof(double));
    return status;
  }
  
  
  void CustomRobot::
  shutdown()
  {
    // nop
  }
  
  
  CustomController::
  CustomController(custom_ds_t * ds)
    : ds_(ds)
  {
  }
  
  
  jspace::Status CustomController::
  setGoal(std::vector<double> const & goal)
  {
    jspace::Status zonk(false, "not implemented");
    return zonk;
  }
  
  
  jspace::Status CustomController::
  getGoal(std::vector<double> & goal) const
  {
    jspace::Status zonk(false, "not implemented");
    return zonk;
  }
  
  
  jspace::Status CustomController::
  getActual(std::vector<double> & actual) const
  {
    jspace::Status zonk(false, "not implemented");
    return zonk;
  }
  
  
  jspace::Status CustomController::
  setGains(std::vector<double> const & kp, std::vector<double> const & kd)
  {
    jspace::Status zonk(false, "not implemented");
    return zonk;
  }
  
  
  jspace::Status CustomController::
  getGains(std::vector<double> & kp, std::vector<double> & kd) const
  {
    jspace::Status zonk(false, "not implemented");
    return zonk;
  }
  
  
  jspace::Status CustomController::
  latch(jspace::Model const & model)
  {
    jspace::Status ok;
    return ok;
  }
  
  
  jspace::Status CustomController::
  computeCommand(jspace::Model const & model, std::vector<double> & tau)
  {
    custom_controller(model, ds_);
    tau.resize(6);
    memcpy(&tau[0], ds_->tau, 6 * sizeof(double));
    jspace::Status ok;
    return ok;
  }
  
}
