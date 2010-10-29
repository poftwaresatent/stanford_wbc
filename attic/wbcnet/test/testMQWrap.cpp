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

/** \file testimp/MQWrap.cpp Unit test of POSIX message queue wrapper. */

#include <wbcnet/imp/MQWrap.hpp>
#include <wbcnet/proxy.hpp>
#include <wbcnet/msg/RobotState.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <sstream>
#include <sys/time.h>
#include <err.h>
#include <stdlib.h>

using namespace wbcnet;
using namespace std;

typedef msg::RobotState<Vector<double>, Matrix<double> > RobotState;


static const endian_mode_t endian_mode(ENDIAN_DETECT);


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


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  idl::CreateSingleton();
  wbcnet::get_logger("wbcnet")->setLevel(wbcnet::Level::getTrace());

  static const int npos(5);
  static const int nvel(4);
  static const int forces_nrows(4);
  static const int forces_ncolumns(4);
  unique_id_t const servo_id(idl::Assign("RobotState"));
  RobotState foo(servo_id, false, npos, nvel, forces_nrows, forces_ncolumns);
  RobotState bar(servo_id, false, npos, nvel, forces_nrows, forces_ncolumns);
  ostringstream os;
  bool ok(true);
  
  init_servo(foo);
  init_servo(bar);
  
  if (foo == bar) {
    ok = false;
    os << "FAILURE: equal ServoData after initialisation\n";
  }
  else
    os << "init test passed\n";
  
  MQWrap queue(true, true);
  Buffer buffer(foo.GetPackSize(), -1);
  com_status cs(queue.Receive(buffer));
  if (COM_OK == cs) {
    ok = false;
    os << "foo.Receive() on empty queue should have FAILED but didn't\n";
  }
  else
    os << "empty queue test passed\n";
  
  if ( ! queue.Open("/testMQWrap", MQWrap::READ_WRITE, 1, foo.GetPackSize(), true))
    exit(EXIT_FAILURE);
  
  proxy_status ps(foo.Pack(buffer, endian_mode));
  if (PROXY_OK != ps) {
    ok = false;
    os << "foo.Pack() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "pack test passed\n";
  
  cs = queue.Send(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "queue.Send() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "send test passed\n";
  
  cs = queue.Receive(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "queue.Receive() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "receive test passed\n";
  
  ps = bar.UnpackHeader(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "bar.UnpackHeader() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "unpack header test passed\n";
  
  ps = bar.UnpackPayload(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "bar.UnpackPayload() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "unpack payload test passed\n";
  
  if (foo != bar) {
    ok = false;
    os << "FAILURE: ServoData should be equal after receive test\n";
  }
  else
    os << "equality test passed\n";
  
  cout << "==================================================\n"
       << os.str();
  if ( ! ok) {
    cout << "detected FAILURES\n";
    return EXIT_FAILURE;    
  }
  cout << "\nALL TESTS PASSED\n\n";
  return EXIT_SUCCESS;
}
