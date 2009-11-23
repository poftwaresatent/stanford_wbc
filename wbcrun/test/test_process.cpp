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

#include <wbcrun/Process.hpp>
#include <wbcrun/service.hpp>
#include <wbcnet/data.hpp>
#include <wbcnet/SPQueue.hpp>
#include <wbcnet/log.hpp>
#include <boost/shared_ptr.hpp>
#include <sys/time.h>
#include <time.h>
#include <err.h>
#include <sstream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcrun"));

namespace wbcrun {
  
  class TestProcess
    : public Process
  {
  public:
    TestProcess(std::string const & name,
		boost::shared_ptr<wbcnet::Sink> sink,
		boost::shared_ptr<wbcnet::Source> source);
    
    /** implements wbcnet::MdxListener */
    virtual int HandleMessagePayload(wbcnet::unique_id_t msg_id);
    
    void Prepare() throw(std::exception);
    void Enqueue();
    
    static wbcnet::unique_id_t const message_id;
    
    ServiceMessage message;
    
    virtual bool Step() throw(std::exception);
    
    void Init() throw(std::exception);
    
  protected:
    boost::shared_ptr<wbcnet::Sink> m_sink;
    boost::shared_ptr<wbcnet::Source> m_source;
  };
  
}

using namespace wbcrun;
using namespace wbcnet;
using namespace std;


int main(int argc, char ** argv)
{
  try {
    boost::shared_ptr<SPQueue> f2b(new SPQueue());
    boost::shared_ptr<SPQueue> b2f(new SPQueue());
    TestProcess foo("foo", f2b, b2f);
    TestProcess bar("bar", b2f, f2b);
    
    foo.Init();
    bar.Init();
    
    warnx("init");
    foo.Prepare();
    usleep(100);
    bar.Prepare();
    if (foo.message == bar.message)
      errx(EXIT_FAILURE, "FAILURE: equal messages after init");
    
    warnx("foo enqueue and step");
    foo.Enqueue();
    foo.Step();
    if (foo.message == bar.message)
      errx(EXIT_FAILURE, "FAILURE: equal messages after foo step");
    
    warnx("bar step");
    bar.Step();
    if (foo.message != bar.message)
      errx(EXIT_FAILURE, "FAILURE: different messages after bar step");
  }
  catch (exception const & ee) {
    errx(EXIT_FAILURE, "exception: %s", ee.what());
  }
  errx(EXIT_SUCCESS, "SUCCESS");
}


namespace wbcrun {
  
  wbcnet::unique_id_t const TestProcess::message_id(2528);

  TestProcess::
  TestProcess(std::string const & name,
	      boost::shared_ptr<wbcnet::Sink> sink,
	      boost::shared_ptr<wbcnet::Source> source)
    : Process(name, 0, -1, wbcnet::ENDIAN_DETECT),
      message(message_id),
      m_sink(sink),
      m_source(source)
  {
  }
  
  
  void TestProcess::
  Init() throw(std::exception)
  {
    AddSink(m_sink.get(), 1);
    AddSource(m_source.get(), 1);
    CreateHandler(message_id, "TestProcess::message", &message);
  }
  
  
  bool TestProcess::
  Step() throw(std::exception)
  {
    Receive();
    Send();
    return true;
  }
  
  
  int TestProcess::
  HandleMessagePayload(wbcnet::unique_id_t msg_id)
  {
    if (message_id != msg_id) {
      LOG_ERROR (logger,
		     "wbcrun::TestProcess::HandleMessagePayload(" << (int) msg_id
		     << "): invalid msg_id, should be " << (int) message_id);
      return 1234;
    }
    
    if (logger->isTraceEnabled()) {
      ostringstream msg;
      msg << "wbcrun::TestProcess::HandleMessagePayload(): got message\n";
      message.Dump(msg, "  ");
      LOG_TRACE (logger, msg.str());
    }
    
    return 0;
  }
  
  
  void TestProcess::
  Enqueue()
  {
    if (logger->isTraceEnabled()) {
      ostringstream msg;
      msg << "wbcrun::TestProcess::Enqueue()\n";
      message.Dump(msg, "  ");
      LOG_TRACE (logger, msg.str());
    }
    EnqueueMessage(m_sink.get(), &message, true, false);
  }
  
  
  void TestProcess::
  Prepare() throw(std::exception)
  {
    struct timeval tv;
    if (-1 == gettimeofday(&tv, 0))
      throw runtime_error("wbcrun::TestProcess::Prepare(): gettimeofday() failed");
    
    message.requestID = static_cast<uint8_t>((tv.tv_usec >> 6) & 0xff);
    
    message.nCodes = static_cast<uint8_t>(tv.tv_usec & 0x0e) + 1;
    message.code.SetNElements(message.nCodes);
    for (uint8_t ii(0); ii < message.nCodes; ++ii)
      message.code[ii] = tv.tv_usec + ii;
    
    message.nRows = static_cast<uint8_t>((tv.tv_usec >> 2) & 0x0e) + 1;
    message.nColumns = static_cast<uint8_t>((tv.tv_usec >> 4) & 0x0e) + 1;
    message.matrix.SetSize(message.nRows, message.nColumns);
    for (int ir(message.nRows - 1); ir >= 0; --ir)
      for (uint8_t ic(0); ic < message.nColumns; ++ic) {
	int const idx(ir * static_cast<int>(message.nColumns) + ic);
	message.matrix.GetElement(idx) = (tv.tv_usec + idx) % 100;
      }
  }
  
}
