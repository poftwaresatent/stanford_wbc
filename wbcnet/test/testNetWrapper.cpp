/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

/**
   \file testNetWrapper.cpp Ad-hoc tests of classes TCPNetWrapper and Muldex.
   
   This test program needs to be run as server in one terminal (or on
   one computer) and then as client in another terminal (or
   computer). Pass the -s option to the one that should run as
   server. Command line arguments:
   - -h print a help message
   - -v verbose mode
   - -s run as server
   - -n "nmsg" quit after processing nmsg iterations (0 means loop forever)
   - -p "port" specify a port (default 9999)
   - -a "addr" specify a network address (default 127.0.0.1)
*/

#include <wbcnet/proxy.hpp>
#include <wbcnet/NetWrapperWrap.hpp>
#include <wbcnet/Muldex.hpp>
#include <wbcnet/DelayHistogram.hpp>
#include <wbcnet/msg/TaskMatrix.hpp>
#include <wbcnet/msg/RobotState.hpp>

#include <wbcnet/log.hpp>

#include <signal.h>
#include <err.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <string.h>

using namespace std;
using namespace wbcnet;

typedef msg::RobotState<Vector<double>, Matrix<double> > RobotState;


static const endian_mode_t endian_mode(ENDIAN_DETECT);
static long const reconnect_usec_sleep(200000); // maybe configurable one day...

static unique_id_t servo_id;
static unique_id_t matrix_id;

class MDX
  : public Muldex
{
public:
  explicit MDX(ostream * dbgos);
  
  virtual int Handle(unique_id_t msg_id, BufferAPI const & buf);
  
  RobotState sdata;
  msg::TaskMatrix mdata;
};


static void usage(ostream & os);
static void parse_options(int argc, char ** argv);
static void run_client();
static void run_server();
static void handle(int signum);
static void cleanup();


static void (*run)() = run_client;

static vector<int> signums;
static in_port_t port(9999);
static bool nonblocking(false);
static string address("127.0.0.1");
static int backlog(0);
static int nmsg(-1);
static MDX * mdx;
static TCPNetWrapper * net_wrapper(0);
static ostream * dbgos(0);


int main(int argc, char ** argv)
{  
  if (0 != atexit(cleanup))
    err(EXIT_FAILURE, "atexit()");
  idl::CreateSingleton();
  servo_id = idl::Assign("servo");
  matrix_id = idl::Assign("matrix");
  
  parse_options(argc, argv);
  
  mdx = new MDX(dbgos);
  
  signums.push_back(SIGHUP);
  signums.push_back(SIGINT);
  signums.push_back(SIGTERM);
  signums.push_back(SIGPIPE);
  for (size_t ii(0); ii < signums.size(); ++ii)
    if (SIG_ERR == signal(signums[ii], handle))
      err(EXIT_FAILURE, "signal(%s)", sys_siglist[signums[ii]]);
  
  run();
}


void run_client()
{
  net_wrapper = new TCPNetWrapper();
  if ( ! net_wrapper->Open(port, address, false, reconnect_usec_sleep)) {
    errx(EXIT_FAILURE,
	 "run_client(): net_wrapper->Open(%d, %s, false, %ld) failed",
	 (int) port,
	 address.c_str(),
	 reconnect_usec_sleep);
  }
  
  cout << "trying to connect to port " << (int) port << " " << flush;
  while ( ! net_wrapper->LazyReconnect(true)) {
    cout << "." << flush;
    usleep(250000);
  }
  cout << "OK\n";
  
  cout << "starting client loop\n";
  int count(0);
  while (true) {
    if ((0 < nmsg) && (count >= nmsg))
      break;
    ++count;
    
    mdx->sdata.jointAngles[0] += 0.1;
    mdx->sdata.jointVelocities[0] -= 0.1;
    mdx->sdata.acquisitionTime.gettimeofday(0);
    
    mdx->mdata.requestID -= 3;
    mdx->mdata.setID = -1;
    mdx->mdata.taskID += 1;
    mdx->mdata.matrixID -= 11;
    mdx->mdata.data.GetElement(0) += 0.101;
    mdx->mdata.acquisitionTime.gettimeofday(0);
    
    cout << "\n==================================================\n"
	 << "client iteration " << count << "\n"
	 << "--------------------------------------------------\n"
	 << "muxing\n";
    
    if (dbgos) mdx->sdata.display(*dbgos, " >> sdata >> ");
    else cout << "  servo state\n";
    
    muldex_status ms(mdx->MuxWait(net_wrapper, mdx->sdata, 100000, &cout));
    if (ms.muldex != muldex_status::SUCCESS)
      errx(EXIT_FAILURE, "mdx->MuxWait() failed with %s",
	   muldex_status_str(ms));
    
    if (dbgos) mdx->mdata.display(*dbgos, " >> mdata >> ");
    else cout << "  task model\n";
    
    ms = mdx->MuxWait(net_wrapper, mdx->mdata, 100000, &cout);
    if (ms.muldex != muldex_status::SUCCESS)
      errx(EXIT_FAILURE, "mdx->MuxWait() failed with %s",
	   muldex_status_str(ms));
    
    cout << "--------------------------------------------------\n"
	 << "demuxing\n";
    
    // XXXX magic numbers      
    ms = mdx->DemuxWait(net_wrapper, 2, 10, 100000, &cout, 0);
    if ((ms.muldex != muldex_status::SUCCESS)
	&& (ms.muldex != muldex_status::TRY_AGAIN))
      errx(EXIT_FAILURE, "mdx->DemuxWait() failed with %s",
	   muldex_status_str(ms));
    
    usleep(250000);
  }
}


void run_server()
{
  net_wrapper = new TCPNetWrapper();
  if ( ! net_wrapper->Open(port, address, true, reconnect_usec_sleep)) {
    errx(EXIT_FAILURE,
	 "run_server(): net_wrapper->Open(%d, %s, true, %ld) failed",
	 (int) port,
	 address.c_str(),
	 reconnect_usec_sleep);
  }
  
  cout << "waiting for connection on port " << (int) port << " " << flush;
  while ( ! net_wrapper->LazyReconnect(true)) {
    cout << "." << flush;
    usleep(250000);
  }
  cout << "OK\n";
  
  cout << "starting server loop\n";
  int count(0);
  while (true) {
    if ((0 < nmsg) && (count >= nmsg))
      break;
    ++count;
    
    cout << "\n==================================================\n"
	 << "server iteration " << count << "\n"
	 << "--------------------------------------------------\n"
	 << "demuxing\n";
    
    muldex_status ms(mdx->DemuxWait(net_wrapper, 2, 10, 100000, &cout, 0)); // XXXX magic numbers
    if ((ms.muldex != muldex_status::SUCCESS)
	&& (ms.muldex != muldex_status::TRY_AGAIN))
      errx(EXIT_FAILURE, "mdx->DemuxWait() failed with %s", muldex_status_str(ms));
    
    mdx->sdata.jointAngles[0] += 0.01;
    mdx->sdata.jointVelocities[0] -= 0.01;
    mdx->sdata.acquisitionTime.gettimeofday(0);
    
    mdx->mdata.requestID += 1;
    mdx->mdata.setID = -1;
    mdx->mdata.taskID -= 2;
    mdx->mdata.matrixID += 3;
    mdx->mdata.data.GetElement(0) -= 0.0101;
    mdx->mdata.acquisitionTime.gettimeofday(0);
    
    cout << "--------------------------------------------------\n"
	 << "muxing\n";
    
    if (dbgos)
      mdx->sdata.display(*dbgos, " >> sdata >> ");
    else
      cout << "  servo state\n";
    
    ms = mdx->MuxWait(net_wrapper, mdx->sdata, 100000, &cout);
    if (ms.muldex != muldex_status::SUCCESS)
      errx(EXIT_FAILURE, "mdx->MuxWait() failed with %s", muldex_status_str(ms));
    
    if (dbgos)
      mdx->mdata.display(*dbgos, " >> mdata >> ");
    else
      cout << "  task model\n";
    
    ms = mdx->MuxWait(net_wrapper, mdx->mdata, 100000, &cout);
    if (ms.muldex != muldex_status::SUCCESS)
      errx(EXIT_FAILURE, "mdx->MuxWait() failed with %s", muldex_status_str(ms));
    
    usleep(250000);
  }
}


void cleanup()
{
  warnx("cleaning up");
  delete net_wrapper;
  delete mdx;
  idl::DestroySingleton();
}


void usage(ostream & os)
{
  os << "options:\n"
     << "   -h        help (this message)\n"
     << "   -v        verbose mode (debug messages)\n"
     << "   -s        run as server (by default we run as client)\n"
     << "   -n <nmsg> quit after processing nmsg iterations (0 means loop forever)\n"
     << "   -p <port> specify a port (default 9999)\n"
     << "   -a <addr> specify a network address (default 127.0.0.1)\n";
}


void parse_options(int argc, char ** argv)
{
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      cerr << argv[0] << ": problem with option '" << argv[ii] << "'\n";
      usage(cerr);
      exit(EXIT_FAILURE);
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(cout);
	exit(EXIT_SUCCESS);
	
      case 'v':
	dbgos = &cerr;
	wbcnet::get_logger("wbcnet")->setLevel(log4cxx::Level::getTrace());
	break;
	
      case 's':
	run = run_server;
	break;
	
      case 'n':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -n requires a nmsg argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> nmsg;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading nmsg argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'p':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -p requires a port argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  int foo;
	  is >> foo;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading port argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	  port = foo;
	}
 	break;

      case 'a':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -a requires an address argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	address = argv[ii];
 	break;
	
      default:
	cerr << argv[0] << ": invalid option '" << argv[ii] << "'\n";
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
}


void handle(int signum)
{
  errx(EXIT_FAILURE, "caught %s", sys_siglist[signum]);
}


MDX::
MDX(ostream * dbgos)
  : Muldex(0, -1, endian_mode),
    sdata(servo_id, false, 1, 1, 1, 1),
    mdata(matrix_id, 1, 1, 1, 1)
{
  sdata.jointAngles[0] = 0;
  sdata.jointVelocities[0] = 0;
  sdata.forces.GetElement(0, 0) = 0;
  mdata.requestID = 17;
  mdata.setID = 217;
  mdata.taskID = 42;
  mdata.matrixID = 21;
  mdata.nRows = 1;
  mdata.nColumns = 1;
  mdata.data.SetSize(1, 1);
  mdata.data.GetElement(0) = 0;
}


int MDX::
Handle(unique_id_t msg_id, BufferAPI const & buf)
{

  // well, "usually" we would return non-zero values instead of
  // killing ourselves

  proxy_status ps;
  if (msg_id == servo_id) {
    ps = sdata.UnpackHeader(buf, endian_mode);
    if (PROXY_OK != ps)
      errx(EXIT_FAILURE, "MDX::Handle(): sdata.UnpackHeader(): %s",
	   proxy_status_str(ps));
    ps = sdata.UnpackPayload(buf, endian_mode);
    if (PROXY_OK != ps)
      errx(EXIT_FAILURE, "MDX::Handle(): sdata.UnpackPayload(): %s",
	   proxy_status_str(ps));
    if (dbgos)
      sdata.display(*dbgos, " << sdata << ");
    else
      cout << "  servo state\n";
  }
  else if (msg_id == matrix_id) {
    ps = mdata.UnpackHeader(buf, endian_mode);
    if (PROXY_OK != ps)
      errx(EXIT_FAILURE, "MDX::Handle(): mdata.UnpackHeader(): %s",
	   proxy_status_str(ps));
    ps = mdata.UnpackPayload(buf, endian_mode);
    if (PROXY_OK != ps)
      errx(EXIT_FAILURE, "MDX::Handle(): mdata.UnpackPayload(): %s",
	   proxy_status_str(ps));
    if (dbgos)
      mdata.display(*dbgos, " << mdata << ");
    else
      cout << "  task model\n";
  }
  else {
    cerr << "No match for message id " << (int) msg_id << "\n"
	 << "  available mappings:\n";
    idl::DumpByName(cerr, "    ");
    try {
      string const name(idl::GetName(msg_id));
      errx(EXIT_FAILURE, "MDX::Handle(): unexpected msg name: %s %d", name.c_str(), msg_id);
    }
    catch (runtime_error const & ee) {
      cerr << "hey, what's this?\n  " << ee.what() << "\n";
    }
    errx(EXIT_FAILURE, "MDX::Handle(): unexpected msg id %d", msg_id);    
  }
  
  return 0;
}
