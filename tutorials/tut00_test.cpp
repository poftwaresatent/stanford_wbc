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
