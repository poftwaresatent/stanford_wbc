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
   \file tut00_test.cpp
   \author Roland Philippsen
   
   A quick test to see whether the planar simulator works as
   expected. It starts by re-initializing the joint state to a swining
   motion, and when you press Toggle it sends zeroed-out torque
   commands to the robot. This makes the robot fall according to
   gravity from whichever position it was in when you press
   Toggle. Pressing Toggle again goes back to the initial mode.
*/

#include "tutsim.hpp"
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <err.h>


static std::string model_filename(TUTROB_XML_PATH_STR);
static boost::shared_ptr<jspace::Model> model;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  model->update(state);
  jspace::Matrix mass_inertia;
  model->getMassInertia(mass_inertia);
  jspace::Vector gravity;
  model->getGravity(gravity);
  
  static size_t counter(0);
  if (0 == (counter % 50)) {
    std::cerr << "wall: " << wall_time_ms << "  sim: " << sim_time_ms << "\n";
    jspace::pretty_print(state.position_, std::cerr, "jpos", "  ");
    jspace::pretty_print(mass_inertia, std::cerr, "mass_inertia", "  ");
    jspace::pretty_print(gravity, std::cerr, "gravity", "  ");
  }
  ++counter;
  
  if (0 == (toggle_count % 2)) {
    static double const aa(M_PI / 2);
    for (int ii(0); ii < state.position_.rows(); ++ii) {
      state.position_[ii] = aa * sin((1.0 + 0.1 * ii) * 1e-3 * wall_time_ms);
      state.velocity_[ii] = 0.0;
    }
    return false;
  }
  
  command = jspace::Vector::Zero(state.position_.rows());
  return true;
}


int main(int argc, char ** argv)
{
  try {
    model.reset(jspace::test::parse_sai_xml_file(model_filename, true));
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  return tutsim::run(servo_cb);
}
