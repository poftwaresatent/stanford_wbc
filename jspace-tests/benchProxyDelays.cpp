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

#include "model_library.hpp"
#include <jspace/State.hpp>
#include <jspace/Model.hpp>
#include <wbcnet/misc/DelayHistogram.hpp>
#include <err.h>
#include <stdlib.h>


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
  
}


int main(int argc, char ** argv)
{
  try {
    jspace::Model * puma_model(jspace::test::create_puma_model());
    jspace::State puma_state(6, 6, 0);
    custom_ds_t ds;
    
    wbcnet::DelayHistogram dhist(4, 5, 0, 2);
    if ( ! (dhist.SetName(0, "convert data")
	    && dhist.SetName(1, "model update")
	    && dhist.SetName(2, "controller")
	    && dhist.SetName(3, "TOTAL"))) {
      errx(EXIT_FAILURE, "dhist.SetName() oops");
    }
    struct timeval t0, t1, t2, t3;
    
    for (int tick(0); tick < 100; ++tick) {
      // Pretend the DS is somehow hooked up to a simulator or whatever...
      for (int ii(0); ii < 6; ++ii) {
	ds.pos[ii] = 0.1 * (tick + ii);
	ds.vel[ii] = 0.05 * (tick - ii);
      }
      ds.tick = tick;
      if (tick % 2) {
	strncpy(ds.hello, "odd", 63);
      }
      else {
	strncpy(ds.hello, "even", 63);
      }
      
      // Update the model according to our state
      if (0 != gettimeofday(&t0, 0)) {
	err(EXIT_FAILURE, "gettimeofday");
      }
      custom_ds_to_state(&ds, puma_state);
      if (0 != gettimeofday(&t1, 0)) {
	err(EXIT_FAILURE, "gettimeofday");
      }
      puma_model->update(puma_state);
      
      // Update controller... this could of course just as easily call
      // something from jspace/controller_library, in which case we'd
      // have to convert from std::vector<double> to double[6], which
      // is a trivial memcpy.
      if (0 != gettimeofday(&t2, 0)) {
	err(EXIT_FAILURE, "gettimeofday");
      }
      custom_controller(*puma_model, &ds);
      
      if (0 != gettimeofday(&t3, 0)) {
	err(EXIT_FAILURE, "gettimeofday");
      }
      
      if ( ! (dhist.StartStop(   0, &t0, &t1)
	      && dhist.StartStop(1, &t1, &t2)
	      && dhist.StartStop(2, &t2, &t3)
	      && dhist.StartStop(3, &t0, &t3))) {
	errx(EXIT_FAILURE, "dhist.StartStop() oops");
      }
      
      // This is where we would send the torques to a robot or the
      // simulator.
      printf("tick: %d (%s)\n"
	     "  pos = %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n"
	     "  vel = %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n"
	     "  tau = %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n"
	     "  eepos = %5.2f %5.2f %5.2f\n",
	     ds.tick, ds.hello,
	     ds.pos[0], ds.pos[1], ds.pos[2], ds.pos[3], ds.pos[4], ds.pos[5], 
	     ds.vel[0], ds.vel[1], ds.vel[2], ds.vel[3], ds.vel[4], ds.vel[5], 
	     ds.tau[0], ds.tau[1], ds.tau[2], ds.tau[3], ds.tau[4], ds.tau[5],
	     ds.eepos[0], ds.eepos[1], ds.eepos[2]);
    }
    
    // Don't forget to clean up after ourselves
    delete puma_model;
    
    printf("\ndelay and computation time measurements\n");
    if ( ! dhist.DumpTable(stdout)) {
      printf("oops, could not print\n");
    }
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
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
    SAITransform eeframe;
    if ( ! model.getGlobalFrame(ee, eeframe)) {
      errx(EXIT_FAILURE, "no end effector frame");
    }
    SAIMatrix6 J;
    if ( ! model.computeJacobian(ee, J)) {
      errx(EXIT_FAILURE, "no end effector Jacobian");
    }
    SAIMatrix A;
    if ( ! model.getMassInertia(A)) {
      errx(EXIT_FAILURE, "no mass inertia");
    }
  
    // Compute torques.
    SAIVector3 delta(eeframe.translation());
    double const dist(delta.magnitude());
    delta /= -dist;
    double strength(dist * dist * 10);
    if (strength > 1) {
      strength = 1;
    }
    SAIVector6 F;
    F[0] = delta[0] * strength;
    F[1] = delta[1] * strength;
    F[2] = delta[2] * strength;
    SAIVector6 tmp, tau;
    J.multiplyTranspose(F, tmp);
    A.multiply(tmp, tau);
  
    // Store the result in our custom data structure.
    if (sizeof(double) != sizeof(Float)) {
      errx(EXIT_FAILURE, "Please recompile with '#define Float double' somewhere in the saimatrix headers.");
    }
    memcpy(ds->tau, tau.dataPtr(), 6 * sizeof(double));
    
    // Store the end-effector position as additional controller output
    // in the custom data structure.
    memcpy(ds->eepos, eeframe.translation().dataPtr(), 3 * sizeof(double));
  }

}
