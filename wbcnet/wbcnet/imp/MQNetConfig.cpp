/* 
 * Copyright (C) 2008 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#include "MQNetConfig.hpp"
#include <wbcnet/MQWrap.hpp>
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>

using namespace std;

#ifdef LINUX
# define MQ_NAME_ROOT "/"
#else
# define MQ_NAME_ROOT ""
#endif

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  static std::string get_mq_name(std::string const & prefix,
				 NetConfig::process_t from_process,
				 NetConfig::process_t to_process)
    throw(runtime_error)
  {
    switch (from_process) {
      
    case NetConfig::SERVO:
      if (NetConfig::MODEL == to_process)
	return MQ_NAME_ROOT + prefix + "_s2m";
      if (NetConfig::USER == to_process)
	return MQ_NAME_ROOT + prefix + "_s2u";
      throw runtime_error("get_mq_name(SERVO, " + sfl::to_string(to_process)
			  + "): invalid to_process");
      
    case NetConfig::MODEL:
      if (NetConfig::SERVO == to_process)
	return MQ_NAME_ROOT + prefix + "_m2s";
      throw runtime_error("get_mq_name(MODEL, " + sfl::to_string(to_process)
			  + "): invalid to_process");
      
    case NetConfig::USER:
      if (NetConfig::SERVO == to_process)
	return MQ_NAME_ROOT + prefix + "_u2s";
      throw runtime_error("get_mq_name(USER, " + sfl::to_string(to_process)
			  + "): invalid to_process");
      
    case NetConfig::DISTANCE:
    case NetConfig::MOTOR:
      throw runtime_error("get_mq_name(" + sfl::to_string(from_process)
			  + ", " + sfl::to_string(to_process)
			  + "): DISTANCE and MOTOR processes are deprecated and will be removed \"real soon now\"");
    }
    
    throw runtime_error("get_mq_name(" + sfl::to_string(from_process)
			+ ", " + sfl::to_string(to_process)
			+ "): invalid from_process / to_process combination");
    return 0;
  }
  
  
  MQNetConfig::
  MQNetConfig(size_t _msg_size, std::string const & _prefix)
    : NetConfig(),
      msg_size(_msg_size),
      prefix(_prefix)
  {
  }
  
  
#ifndef WBCNET_HAVE_MQUEUE
  
  
  wbcnet::Channel * MQNetConfig::
  CreateChannel(process_t from_process,
		process_t to_process) const
    throw(runtime_error)
  {
    throw runtime_error("wbcnet::MQNetConfig::CreateChannel(): POSIX message queues not supported");
    return 0;
  }
  
  
#else // WBCNET_HAVE_MQUEUE
  
  
  static wbcnet::MQWrap * CreateMQWrap(std::string const & prefix,
				       NetConfig::process_t from_process,
				       NetConfig::process_t to_process,
				       size_t msg_size,
				       wbcnet::MQWrap::mode_t mode)
    throw(runtime_error)
  {
    long maxmsg(1);
    if ((from_process == NetConfig::USER)
	|| (from_process == NetConfig::MODEL))
      maxmsg = 10;
    std::string const mq_name(get_mq_name(prefix, from_process, to_process));
    
    if (logger->isTraceEnabled()) {
      ostringstream msg;
      msg << "wbcnet::MQNetConfig::CreateMQWrap(" << prefix << ", " << from_process << ", "
	  << to_process << "): opening \"" << mq_name << "\" with max " << maxmsg
	  << "messages for ";
      switch (mode) {
      case wbcnet::MQWrap::READ_ONLY:
	msg << "reading"; break;
      case wbcnet::MQWrap::READ_WRITE:
	msg << "reading and writing"; break;
      case wbcnet::MQWrap::WRITE_ONLY:
	msg << "writing"; break;
      default:
	msg << "OOPS dunno that mode (" << mode << ")"; break;
      }
      LOG_TRACE (logger, msg.str());
    }
    
    wbcnet::MQWrap * mqw(new wbcnet::MQWrap(true, true));
    if ( ! mqw->Open(mq_name, mode, maxmsg, msg_size, true)) {
      delete mqw;
      throw runtime_error("MQNetConfig::CreateMQWrap(): mqw->Open(" + mq_name
			  + ",...) failed");
    }
    return mqw;
  }
  
  
  wbcnet::Channel * MQNetConfig::
  CreateChannel(process_t from_process,
		process_t to_process) const
    throw(runtime_error)
  {
    wbcnet::Sink * sink(CreateMQWrap(prefix, from_process, to_process, msg_size,
				     wbcnet::MQWrap::WRITE_ONLY));
    try {
      wbcnet::Source * source(CreateMQWrap(prefix, to_process, from_process, msg_size,
					   wbcnet::MQWrap::READ_ONLY));
      return new wbcnet::ProxyChannel(sink, true, source, true);
    }
    catch (runtime_error const & ee) {
      delete sink;
      throw ee;
    }
    return 0;
  }
  
  
#endif // WBCRUN_HAVE_MQUEUE
  
}
