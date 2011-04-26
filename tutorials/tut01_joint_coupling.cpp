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
