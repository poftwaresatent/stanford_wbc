/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include <wbcnet/Muldex.hpp>
#include <wbcnet/imp/SPQueue.hpp>
#include <wbcnet/msg/RobotState.hpp>
#include <wbcnet/msg/TaskSpec.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <err.h>
#include <unistd.h>

using namespace wbcnet;
using namespace std;

typedef msg::RobotState<Vector<double>, Matrix<double> > RobotState;

static const endian_mode_t endian_mode(ENDIAN_DETECT);

namespace {
  
  // on purpose we do not handle task specs to trigger (and test)
  // error handling code
  class Listener: public MdxListener
  {
  public:
    explicit Listener(ostream & _os)
      : servo_proxy(idl::GetID("RobotState"), false, 3, 4, 4, 2),
	os(_os)
    {
    }
    
    virtual int HandleMessagePayload(unique_id_t msg_id) {
      if (idl::GetID("RobotState") == msg_id) {
	cout << "HandleMessagePayload(" << (int) msg_id << ") matches RobotState\n";
	return 0;
      }
      os << "HandleMessagePayload(" << (int) msg_id << ") unknown ID \"" << idl::GetName(msg_id)
	 << "\"\n  does not match RobotState\n";
      return 17;
    }
    
    RobotState servo_proxy;
    ostream & os;
  };
  
}


static void cleanup()
{
  idl::DestroySingleton();
}


static void init_servo(RobotState & sd)
{
  if ( ! sd.acquisitionTime.gettimeofday(0))
    err(EXIT_FAILURE, "init_servo(): gettimeofday()");
  
  cout << "init_servo():\n ";
  for (int ii(0); ii < sd.jointAngles.GetLength(); ++ii) {
    sd.jointAngles[ii] = sd.acquisitionTime.tv_usec + ii;
    cout << " " << sd.jointAngles[ii];
  }
  cout << "\n ";
  for (int ii(0); ii < sd.jointVelocities.GetLength(); ++ii) {
    sd.jointVelocities[ii] = sd.acquisitionTime.tv_usec + ii;
    cout << " " << sd.jointVelocities[ii];
  }
  cout << "\n ";
  for (int irow(0); irow < sd.forces.NRows(); ++irow) {
    for (int icol(0); icol < sd.forces.NColumns(); ++icol) {
      sd.forces.GetElement(irow, icol) = sd.acquisitionTime.tv_usec + 20 * irow + icol;
      cout << " " << sd.forces.GetElement(irow, icol);
    }
    cout << "\n";
  }
}


static void init_task_spec(msg::TaskSpec & tsd)
{
  usleep(1000);
  
  timestamp foo;
  if ( ! foo.gettimeofday(0))
    err(EXIT_FAILURE, "init_task_spec(): gettimeofday()");
  
  tsd.requestID  = (foo.tv_usec % 256) & 255;
  tsd.behaviorID = tsd.requestID + 1;
  
  cout << "init_task_spec():\n";
  tsd.display(cout, "  ");
}


static bool mux_servo(SPQueue & queue, Muldex & mdx, ostream & os)
{
  RobotState servo_proxy(idl::GetID("RobotState"), false, 3, 4, 4, 2);
  init_servo(servo_proxy);
  muldex_status const ms(mdx.Mux(&queue, servo_proxy));
  if (muldex_status::SUCCESS != ms.muldex) {
    os << "mux_servo(): mdx.Mux() FAILED with " << muldex_status_str(ms) << "\n";
    return false;
  }
  return true;
}


static bool mux_task(SPQueue & queue, Muldex & mdx, ostream & os)
{
  msg::TaskSpec task_spec_proxy(idl::GetID("msg::TaskSpec"));
  init_task_spec(task_spec_proxy);
  muldex_status const ms(mdx.Mux(&queue, task_spec_proxy));
  if (muldex_status::SUCCESS != ms.muldex) {
    os << "mux_task(): mdx.Mux() FAILED with " << muldex_status_str(ms) << "\n";
    return false;
  }
  return true;
}


static bool demux(SPQueue & queue, Muldex & mdx, ostream & os)
{
  muldex_status const ms(mdx.DemuxOne(&queue));
  if (muldex_status::SUCCESS != ms.muldex) {
    os << "mux_task(): mdx.DemuxOne() FAILED with " << muldex_status_str(ms) << "\n";
    return false;
  }
  return true;  
}


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  idl::CreateAutoAssignSingleton();
  wbcnet::configure_logging();
  
  ostringstream os;
  bool ok(true);
  Listener listener(os);
  MdxDispatcher mdx(0, -1, endian_mode);
  
  { // no handler, demux should fail
    SPQueue queue;
    if ( ! mux_servo(queue, mdx, os)) {
      ok = false;
      os << "no handler, demux should fail: could not mux_servo()\n";
    }
    if (demux(queue, mdx, cout)) {
      ok = false;
      os << "no handler, demux should fail: demux() did not fail\n";
    }
  }
  
  mdx.SetHandler(idl::GetID("RobotState"),
		 new ProxyHandler("servo", listener.servo_proxy, true, &listener));
  { // with servo handler
    SPQueue queue;
    if ( ! mux_servo(queue, mdx, os)) {
      ok = false;
      os << "with servo handler: could not mux_servo()\n";
    }
    if ( ! demux(queue, mdx, os)) {
      ok = false;
      os << "with servo handler: demux() failed on servo\n";
    }
    if ( ! mux_task(queue, mdx, os)) {
      ok = false;
      os << "with servo handler: could not mux_task()\n";
    }
    if (demux(queue, mdx, cout)) {
      ok = false;
      os << "with servo handler: demux() did not fail on task\n";
    }
  }
  
  msg::TaskSpec foo(idl::GetID("msg::TaskSpec"));
  mdx.SetHandler(idl::GetID("msg::TaskSpec"),
		 new ProxyHandler("task spec", foo, true, &listener));
  { // with invalid task handler
    SPQueue queue;
    if ( ! mux_servo(queue, mdx, os)) {
      ok = false;
      os << "with invalid task handler: could not mux_servo()\n";
    }
    if ( ! demux(queue, mdx, os)) {
      ok = false;
      os << "with invalid task handler: demux() failed on servo\n";
    }
    if ( ! mux_task(queue, mdx, os)) {
      ok = false;
      os << "with invalid task handler: could not mux_task()\n";
    }
    if (demux(queue, mdx, cout)) {
      ok = false;
      os << "with invalid task handler: demux() did not fail on task\n";
    }
  }
  
  cout << "==================================================\n"
       << os.str();
  if ( ! ok) {
    cout << "detected FAILURES\n";
    return EXIT_FAILURE;    
  }
  cout << "\nALL TESTS PASSED\n\n";
  return EXIT_SUCCESS;
}
