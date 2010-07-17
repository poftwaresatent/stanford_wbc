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

#ifdef WBCNET_HAVE_MQUEUE
# include <wbcnet/imp/MQWrap.hpp>
#endif // WBCNET_HAVE_MQUEUE

#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <vector>

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
  MQNetConfig(bool blocking, size_t msg_size, std::string const & prefix)
    : NetConfig(),
      m_blocking(blocking),
      m_msg_size(msg_size),
      m_prefix(prefix)
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
  
  
  wbcnet::Channel * MQNetConfig::
  CreateChannel(std::string const & connection_spec) const
    throw(std::runtime_error)
  {
    throw runtime_error("wbcnet::MQNetConfig::CreateChannel(): POSIX message queues not supported");
    return 0;
  }
  
  
#else // WBCNET_HAVE_MQUEUE
  
  
  static wbcnet::MQWrap * CreateMQWrap(std::string const & prefix,
				       NetConfig::process_t from_process,
				       NetConfig::process_t to_process,
				       size_t msg_size,
				       wbcnet::MQWrap::mode_t mode,
				       bool blocking)
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
      if (blocking) {
	msg << " in BLOCKING mode";
      }
      else {
	msg << " in NON-blocking mode";
      }
      LOG_TRACE (logger, msg.str());
    }
    
    wbcnet::MQWrap * mqw(new wbcnet::MQWrap(true, true));
    if ( ! mqw->Open(mq_name, mode, maxmsg, msg_size, ! blocking)) {
      delete mqw;
      throw runtime_error("MQNetConfig::CreateMQWrap(): mqw->Open(" + mq_name
			  + ", " + sfl::to_string(mode) + ", " + sfl::to_string(maxmsg)
			  + ", " + sfl::to_string(msg_size) + ", " + sfl::to_string( ! blocking)
			  + ") failed");
    }
    return mqw;
  }
  
  
  wbcnet::Channel * MQNetConfig::
  CreateChannel(process_t from_process,
		process_t to_process) const
    throw(runtime_error)
  {
    wbcnet::Sink * sink(CreateMQWrap(m_prefix, from_process, to_process, m_msg_size,
				     wbcnet::MQWrap::WRITE_ONLY, m_blocking));
    try {
      wbcnet::Source * source(CreateMQWrap(m_prefix, to_process, from_process, m_msg_size,
					   wbcnet::MQWrap::READ_ONLY, m_blocking));
      return new wbcnet::ProxyChannel(sink, true, source, true);
    }
    catch (runtime_error const & ee) {
      delete sink;
      throw ee;
    }
    return 0;
  }
  
  
  wbcnet::Channel * MQNetConfig::
  CreateChannel(std::string const & connection_spec) const
    throw(std::runtime_error)
  {
    LOG_DEBUG (logger, "MQNetConfig::CreateChannel(" << connection_spec << ") ");
    
    vector<string> token;
    sfl::tokenize(connection_spec, ':', token);
    
    string name_wr;
    sfl::token_to(token, 0, name_wr);
    if (name_wr.empty()) {
      throw runtime_error("MQNetConfig::CreateChannel(" + connection_spec
			  + "): invalid connection_spec, I need at least a queue name to work with here...");
    }
    string name_rd;
    sfl::token_to(token, 1, name_rd);
    if (name_rd.empty()) {
      name_rd = name_wr;
    }
    long maxmsg(-1);
    sfl::token_to(token, 2, maxmsg);
    if (maxmsg < 0) {
      maxmsg = 10;
    }
    
    wbcnet::MQWrap::mode_t mode(wbcnet::MQWrap::READ_WRITE);
    if (name_wr != name_rd) {
      mode = wbcnet::MQWrap::WRITE_ONLY;
    }
    
    name_wr = MQ_NAME_ROOT + m_prefix + "_" + name_wr;
    name_rd = MQ_NAME_ROOT + m_prefix + "_" + name_rd;
    
    wbcnet::MQWrap * sink(new wbcnet::MQWrap(true, true));
    if ( ! sink->Open(name_wr, mode, maxmsg, m_msg_size, ! m_blocking)) {
      delete sink;
      ostringstream msg;
      msg << "MQNetConfig::CreateChannel(): sink->Open(" << name_wr << ", " << mode << ", " << maxmsg
	  << ", " << m_msg_size << ", " << sfl::to_string(! m_blocking) << ") FAILED";
      throw runtime_error(msg.str());
    }
    LOG_INFO (logger,
	      "MQNetConfig::CreateChannel(): sink->Open(" << name_wr << ", " << mode << ", " << maxmsg
	      << ", " << m_msg_size << ", " << sfl::to_string(! m_blocking) << ") succeeded");
    
    if (name_wr != name_rd) {
      mode = wbcnet::MQWrap::READ_ONLY;
    }
    wbcnet::MQWrap * source(new wbcnet::MQWrap(true, true));
    if ( ! source->Open(name_rd, mode, maxmsg, m_msg_size, ! m_blocking)) {
      delete sink;
      delete source;
      ostringstream msg;
      msg << "MQNetConfig::CreateChannel(): source->Open(" << name_rd << ", " << mode << ", " << maxmsg
	  << ", " << m_msg_size << ", " << sfl::to_string(! m_blocking) << ") FAILED";
      throw runtime_error(msg.str());
    }
    LOG_INFO (logger,
	      "MQNetConfig::CreateChannel(): source->Open(" << name_rd << ", " << mode << ", " << maxmsg
	      << ", " << m_msg_size << ", " << sfl::to_string(! m_blocking) << ") succeeded");
    
    return new wbcnet::ProxyChannel(sink, true, source, true);
  }
  
#endif // WBCRUN_HAVE_MQUEUE
  
}
