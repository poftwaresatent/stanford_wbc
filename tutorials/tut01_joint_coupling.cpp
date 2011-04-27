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
   \file tut01_joint_coupling.cpp
   \author Roland Philippsen
   
   A tutorial which illustrates the dynamic coupling between joints of
   a kinematic chain. The simulation starts out as a PD control to the
   zero position. When you press Toggle, it sends a torque command
   which is zero everyhere except for the last joint, which receives a
   sinusoidal open-loop torque. The resulting motion shows that even
   though only one joint has non-zero torque, the entire chain starts
   to move. Clicking Toggle again switches back to the initial mode
   and then the cycle repeats.
*/

#include "tutsim.hpp"


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  if (0 == (toggle_count % 2)) {
    command = -400.0 * state.position_ - 20.0 * state.velocity_;
  }
  else {
    command = jspace::Vector::Zero(state.position_.rows());
    int const idx(command.rows() - 1);
    double dq_des(15.0);
    if (fmod(sim_time_ms, 4e3) > 2e3) {
      dq_des = -dq_des;
    }
    command[idx] = dq_des - 2.0 * state.velocity_[idx];
  }
  
  static size_t iteration(0);
  if (0 == (iteration++ % 100)) {
    std::cerr << "sim_time_ms: " << sim_time_ms << "\n";
    jspace::pretty_print(state.position_, std::cerr, "jpos", "  ");
    jspace::pretty_print(state.velocity_, std::cerr, "jvel", "  ");
    jspace::pretty_print(command, std::cerr, "command", "  ");
  }
  
  return true;
}


int main(int argc, char ** argv)
{
  return tutsim::run(servo_cb);
}
