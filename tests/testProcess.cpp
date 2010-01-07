/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file testProcess.cpp
   \author Roland Philippsen
   \note Originally Copyright (c) 2008 Roland Philippsen, released under a BSD license.
*/

#include <wbc/bin/Process.hpp>
#include <wbcnet/msg/Service.hpp>
#include <wbcnet/data.hpp>
#include <wbcnet/imp/SPQueue.hpp>
#include <wbcnet/log.hpp>
#include <sys/time.h>
#include <time.h>
#include <err.h>
#include <sstream>
#include <stdlib.h>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc.tests"));

namespace wbc {
  
  class TestProcess
    : public Process
  {
  public:
    TestProcess(std::string const & name,
		wbcnet::Sink * sink,
		wbcnet::Source * source);
    
    /** implements wbcnet::MdxListener */
    virtual int HandleMessagePayload(wbcnet::unique_id_t msg_id);
    
    void Prepare() throw(std::exception);
    void Enqueue();
    
    static wbcnet::unique_id_t const message_id;
    
    wbcnet::msg::Service message;
    
    virtual bool Step() throw(std::exception);
    
    void Init() throw(std::exception);
    
  protected:
    wbcnet::Sink * m_sink;
    wbcnet::Source * m_source;
  };
  
}

using namespace wbc;
using namespace wbcnet;
using namespace std;


int main(int argc, char ** argv)
{
  try {
    SPQueue f2b, b2f;
    TestProcess foo("foo", &f2b, &b2f);
    TestProcess bar("bar", &b2f, &f2b);
    
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


namespace wbc {
  
  wbcnet::unique_id_t const TestProcess::message_id(2528);

  TestProcess::
  TestProcess(std::string const & name,
	      wbcnet::Sink * sink,
	      wbcnet::Source * source)
    : Process(name, 0, -1, wbcnet::ENDIAN_DETECT),
      message(message_id),
      m_sink(sink),
      m_source(source)
  {
  }
  
  
  void TestProcess::
  Init() throw(std::exception)
  {
    AddSink(m_sink, 1);
    AddSource(m_source, 1);
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
		 "wbcnet::TestProcess::HandleMessagePayload(" << (int) msg_id
		 << "): invalid msg_id, should be " << (int) message_id);
      return 1234;
    }
    
    if (logger->isTraceEnabled()) {
      ostringstream msg;
      msg << "wbcnet::TestProcess::HandleMessagePayload(): got message\n";
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
      msg << "wbcnet::TestProcess::Enqueue()\n";
      message.Dump(msg, "  ");
      LOG_TRACE (logger, msg.str());
    }
    EnqueueMessage(m_sink, &message, true, false);
  }
  
  
  void TestProcess::
  Prepare() throw(std::exception)
  {
    struct timeval tv;
    if (-1 == gettimeofday(&tv, 0))
      throw runtime_error("wbcnet::TestProcess::Prepare(): gettimeofday() failed");
    
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
