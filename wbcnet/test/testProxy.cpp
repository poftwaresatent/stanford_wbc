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

/** \file testProxy.cpp Unit test for Proxy (sub)classes. */

#include <wbcnet/SPQueue.hpp>
#include <wbcnet/proxy.hpp>
#include <wbcnet/msg/RobotState.hpp>
#include <wbcnet/msg/StringList.hpp>
#include <wbcnet/msg/TaskSpec.hpp>
#include <wbcnet/msg/TaskMatrix.hpp>
#include <wbcnet/msg/Status.hpp>
#include <wbcnet/msg/ServoCommand.hpp>
#include <wbcnet/log.hpp>
#include <iostream>
#include <sstream>
#include <list>
#include <sys/time.h>
#include <err.h>
#include <unistd.h>
#include <stdlib.h>

using namespace wbcnet;
using namespace std;

typedef msg::RobotState<Vector<double>, Matrix<double> > RobotState;
typedef msg::ServoCommand<Vector<double> > ServoCommand;
using msg::StringList;

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


static void init_task_matrix(msg::TaskMatrix & tmd)
{
  if ( ! tmd.acquisitionTime.gettimeofday(0))
    err(EXIT_FAILURE, "init_task_matrix(): gettimeofday()");
  
  cout << "init_task_matrix():\n";
  for (int ii(0); ii < tmd.data.NRows(); ++ii) {
    cout << " ";
    for (int jj(0); jj < tmd.data.NColumns(); ++jj) {
      int const kk(ii + jj * tmd.data.NRows());
      tmd.data.GetElement(kk) = tmd.acquisitionTime.tv_usec + kk;
      cout << " " << tmd.data.GetElement(kk);
    }
    cout << "\n";
  }
}


static void init_status(msg::Status & sp)
{
  usleep(1000);
  
  timestamp foo;
  if ( ! foo.gettimeofday(0))
    err(EXIT_FAILURE, "init_status(): gettimeofday()");
  
  sp.status = (foo.tv_usec % 256) & 255;
  
  cout << "init_status():\n";
  sp.display(cout, "  ");
}


static void init_cmd(ServoCommand & cmd)
{
  if ( ! cmd.acquisitionTime.gettimeofday(0))
    err(EXIT_FAILURE, "init_cmd(): gettimeofday()");
  
  cout << "init_cmd(): " << cmd.command.GetLength() << " command\n";
  for (int ii(0); ii < cmd.command.GetLength(); ++ii) {
    cmd.command[ii] = cmd.acquisitionTime.tv_usec + ii;
    cout << " " << cmd.command[ii];
  }
  cout << "\n";
}


static bool test_servo(uint8_t npos, uint8_t nvel,
		       uint8_t forces_nrows, uint8_t forces_ncolumns,
		       ostream & os)
{
  unique_id_t const servo_id(idl::Assign("RobotState"));
  RobotState foo(servo_id, false, npos, nvel, forces_nrows, forces_ncolumns);
  RobotState bar(servo_id, false, npos, nvel, forces_nrows, forces_ncolumns);
  init_servo(foo);
  init_servo(bar);
  
  bool ok(true);
  
  if (foo == bar) {
    ok = false;
    os << "test_servo(): FAILURE: equal ServoData after initialisation\n";
  }
  else
    os << "test_servo(): init test passed\n";
  
  SPQueue queue;
  Buffer buffer(foo.GetPackSize(), -1);
  com_status cs(queue.Receive(buffer));
  if (COM_OK == cs) {
    ok = false;
    os << "test_servo(): queue.Receive() on empty queue should have FAILED but didn't\n";
  }
  else
    os << "test_servo(): empty queue test passed\n";
  
  proxy_status ps(foo.Pack(buffer, endian_mode));
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_servo(): foo.Pack() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_servo(): pack test passed\n";
  
  cs = queue.Send(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_servo(): queue.Send() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_servo(): send test passed\n";
  
  cs = queue.Receive(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_servo(): queue.Receive() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_servo(): receive test passed\n";
  
  ps = bar.UnpackHeader(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_servo(): bar.UnpackHeader() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_servo(): unpack header test passed\n";
  
  ps = bar.UnpackPayload(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_servo(): bar.UnpackPayload() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_servo(): unpack payload test passed\n";
  
  if (foo != bar) {
    ok = false;
    os << "test_servo(): FAILURE: ServoData should be equal after receive test\n";
  }
  else
    os << "test_servo(): equality test passed\n";
  
  return ok;
}


static bool test_task_spec(ostream & os)
{
  idl::Assign("msg::TaskSpec");
  msg::TaskSpec foo(idl::GetID("msg::TaskSpec"));
  msg::TaskSpec bar(idl::GetID("msg::TaskSpec"));
  init_task_spec(foo);
  init_task_spec(bar);
  
  bool ok(true);
  
  if (foo == bar) {
    ok = false;
    os << "test_task_spec(): FAILURE: equal TaskSpecData after initialisation\n";
  }
  else
    os << "test_task_spec(): init test passed\n";
  
  SPQueue queue;
  Buffer buffer(foo.GetPackSize(), -1);
  com_status cs(queue.Receive(buffer));
  if (COM_OK == cs) {
    ok = false;
    os << "test_task_spec(): queue.Receive() on empty queue should have FAILED but didn't\n";
  }
  else
    os << "test_task_spec(): empty queue test passed\n";
  
  proxy_status ps(foo.Pack(buffer, endian_mode));
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_task_spec(): foo.Pack() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_task_spec(): pack test passed\n";
  
  cs = queue.Send(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_task_spec(): queue.Send() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_task_spec(): send test passed\n";
  
  cs = queue.Receive(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_task_spec(): queue.Receive() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_task_spec(): receive test passed\n";
  
  ps = bar.UnpackHeader(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_task_spec(): bar.UnpackHeader() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_task_spec(): unpack header test passed\n";
  
  ps = bar.UnpackPayload(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_task_spec(): bar.UnpackPayload() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_task_spec(): unpack payload test passed\n";
  
  if (foo != bar) {
    ok = false;
    os << "test_task_spec(): FAILURE: ServoData should be equal after receive test\n";
  }
  else
    os << "test_task_spec(): equality test passed\n";
  
  return ok;
}


static bool test_task_matrix(uint8_t ndof, ostream & os)
{
  idl::Assign("msg::TaskMatrix");
  msg::TaskMatrix foo(idl::GetID("msg::TaskMatrix"), ndof, ndof, ndof, ndof);
  msg::TaskMatrix bar(idl::GetID("msg::TaskMatrix"), ndof, ndof, ndof, ndof);
  init_task_matrix(foo);
  init_task_matrix(bar);
  
  bool ok(true);
  
  if (foo == bar) {
    ok = false;
    os << "test_task_matrix(): FAILURE: equal msg::TaskMatrix after initialisation\n";
  }
  else
    os << "test_task_matrix(): init test passed\n";
  
  SPQueue queue;
  Buffer buffer(foo.GetPackSize(), -1);
  com_status cs(queue.Receive(buffer));
  if (COM_OK == cs) {
    ok = false;
    os << "test_task_matrix(): queue.Receive() on empty queue should have FAILED but didn't\n";
  }
  else
    os << "test_task_matrix(): empty queue test passed\n";
  
  proxy_status ps(foo.Pack(buffer, endian_mode));
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_task_matrix(): foo.Pack() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_task_matrix(): pack test passed\n";
  
  cs = queue.Send(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_task_matrix(): queue.Send() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_task_matrix(): send test passed\n";
  
  cs = queue.Receive(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_task_matrix(): queue.Receive() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_task_matrix(): receive test passed\n";
  
  ps = bar.UnpackHeader(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_task_matrix(): bar.UnpackHeader() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_task_matrix(): unpack header test passed\n";
  
  ps = bar.UnpackPayload(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_task_matrix(): bar.UnpackPayload() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_task_matrix(): unpack payload test passed\n";
  
  if (foo != bar) {
    ok = false;
    os << "test_task_matrix(): FAILURE: ServoData should be equal after receive test\n";
  }
  else
    os << "test_task_matrix(): equality test passed\n";
  
  return ok;
}


static bool test_status(ostream & os)
{
  idl::Assign("msg::Status");
  msg::Status foo(idl::GetID("msg::Status"));
  msg::Status bar(idl::GetID("msg::Status"));
  init_status(foo);
  init_status(bar);
  
  bool ok(true);
  
  if (foo == bar) {
    ok = false;
    os << "test_status(): FAILURE: equal msg::Status after initialisation\n";
  }
  else
    os << "test_status(): init test passed\n";
  
  SPQueue queue;
  Buffer buffer(foo.GetPackSize(), -1);
  com_status cs(queue.Receive(buffer));
  if (COM_OK == cs) {
    ok = false;
    os << "test_status(): queue.Receive() on empty queue should have FAILED but didn't\n";
  }
  else
    os << "test_status(): empty queue test passed\n";
  
  proxy_status ps(foo.Pack(buffer, endian_mode));
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_status(): foo.Pack() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_status(): pack test passed\n";
  
  cs = queue.Send(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_status(): queue.Send() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_status(): send test passed\n";
  
  cs = queue.Receive(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_status(): queue.Receive() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_status(): receive test passed\n";
  
  ps = bar.UnpackHeader(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_status(): bar.UnpackHeader() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_status(): unpack header test passed\n";
  
  ps = bar.UnpackPayload(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_status(): bar.UnpackPayload() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_status(): unpack payload test passed\n";
  
  if (foo != bar) {
    ok = false;
    os << "test_status(): FAILURE: ServoData should be equal after receive test\n";
  }
  else
    os << "test_status(): equality test passed\n";
  
  return ok;
}


static bool test_cmd(uint8_t ncommands, ostream & os)
{
  idl::Assign("ServoCommand");
  ServoCommand foo(idl::GetID("ServoCommand"), false, ncommands);
  ServoCommand bar(idl::GetID("ServoCommand"), false, ncommands);
  init_cmd(foo);
  init_cmd(bar);
  
  bool ok(true);
  
  if (foo == bar) {
    ok = false;
    os << "test_cmd(): FAILURE: equal CommandData after initialisation\n";
  }
  else
    os << "test_cmd(): init test passed\n";
  
  SPQueue queue;
  Buffer buffer(foo.GetPackSize(), -1);
  com_status cs(queue.Receive(buffer));
  if (COM_OK == cs) {
    ok = false;
    os << "test_cmd(): queue.Receive() on empty queue should have FAILED but didn't\n";
  }
  else
    os << "test_cmd(): empty queue test passed\n";
  
  proxy_status ps(foo.Pack(buffer, endian_mode));
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_cmd(): foo.Pack() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_cmd(): pack test passed\n";
  
  cs = queue.Send(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_cmd(): queue.Send() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_cmd(): send test passed\n";
  
  cs = queue.Receive(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_cmd(): queue.Receive() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_cmd(): receive test passed\n";
  
  ps = bar.UnpackHeader(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_cmd(): bar.UnpackHeader() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_cmd(): unpack header test passed\n";
  
  ps = bar.UnpackPayload(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_cmd(): bar.UnpackPayload() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_cmd(): unpack payload test passed\n";
  
  if (foo != bar) {
    ok = false;
    os << "test_cmd(): FAILURE: CommandData should be equal after receive test\n";
  }
  else
    os << "test_cmd(): equality test passed\n";
  
  return ok;
}


static bool test_strlist(ostream & os)
{
  list<string> check, crosscheck;
  check.push_back("foo");
  check.push_back("bar");
  check.push_back("");
  check.push_back("baz");
  crosscheck.push_back("noenoe");
  crosscheck.push_back("42");
  
  idl::Assign("StringList");
  StringList foo(idl::GetID("StringList"));
  StringList bar(idl::GetID("StringList"));
  
  foo.append(check.begin(), check.end());
  bar.append(crosscheck.begin(), crosscheck.end());
  
  bool ok(true);
  
  if (foo == bar) {
    ok = false;
    os << "test_strlist(): FAILURE: equal StringList after initialisation\n";
  }
  else
    os << "test_strlist(): init test passed\n";
  
  SPQueue queue;
  Buffer buffer(foo.GetPackSize(), -1);
  com_status cs(queue.Receive(buffer));
  if (COM_OK == cs) {
    ok = false;
    os << "test_strlist(): queue.Receive() on empty queue should have FAILED but didn't\n";
  }
  else
    os << "test_strlist(): empty queue test passed\n";
  
  proxy_status ps(foo.Pack(buffer, endian_mode));
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_strlist(): foo.Pack() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_strlist(): pack test passed\n";
  
  cs = queue.Send(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_strlist(): queue.Send() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_strlist(): send test passed\n";
  
  cs = queue.Receive(buffer);
  if (COM_OK != cs) {
    ok = false;
    os << "test_strlist(): queue.Receive() FAILED with " << com_status_str(cs) << "\n";
  }
  else
    os << "test_strlist(): receive test passed\n";
  
  ps = bar.UnpackHeader(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_strlist(): bar.UnpackHeader() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_strlist(): unpack header test passed\n";
  
  ps = bar.UnpackPayload(buffer, endian_mode);
  if (PROXY_OK != ps) {
    ok = false;
    os << "test_strlist(): bar.UnpackPayload() FAILED with " << proxy_status_str(ps) << "\n";
  }
  else
    os << "test_strlist(): unpack payload test passed\n";
  
  if (foo != bar) {
    ok = false;
    os << "test_strlist(): FAILURE: StringList should be equal after receive test\n"
       << "  foo:\n";
    foo.display(os, "    ");
    os << "  bar:\n";
    bar.display(os, "    ");
  }
  else
    os << "test_strlist(): equality test passed\n";
  
  return ok;
}


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  idl::CreateSingleton();
  wbcnet::get_logger("wbcnet")->setLevel(log4cxx::Level::getTrace());
  
  ostringstream os;
  bool ok(true);
  
  ok = test_servo(4, 3, 2, 1, os);
  ok = test_task_spec(os);
  ok = test_task_matrix(3, os);
  ok = test_status(os);
  ok = test_cmd(2, os);
  ok = test_strlist(os);
  
  cout << "==================================================\n"
       << os.str();
  if ( ! ok) {
    cout << "detected FAILURES\n";
    return EXIT_FAILURE;    
  }
  cout << "\nALL TESTS PASSED\n\n";
  return EXIT_SUCCESS;
}
