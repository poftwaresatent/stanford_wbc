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

#include "SPQNetConfig.hpp"
#include <wbcnet/SPQueue.hpp>
#include <wbcnet/log.hpp>
#include <wbcnet/strutil.hpp>

using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  SPQNetConfig::
  SPQNetConfig()
    : NetConfig()
  {
  }
  
  
  wbcnet::Channel * SPQNetConfig::
  CreateChannel(std::string const & from_process,
		std::string const & to_process) const
  {
    bool own_sink(false);
    wbcnet::SPQueue * sink(GetSPQueue(from_process, to_process));
    if ( ! sink) {
      sink = CreateSPQueue(from_process, to_process);
      own_sink = true;
    }
    
    bool own_source(false);
    wbcnet::SPQueue * source(GetSPQueue(to_process, from_process));
    if ( ! source) {
      source = CreateSPQueue(to_process, from_process);
      own_source = true;
    }
    
    return new wbcnet::ProxyChannel(sink, own_sink, source, own_source);
  }
  
  
  wbcnet::Channel * SPQNetConfig::
  CreateChannel(std::string const & connection_spec) const
    throw(std::runtime_error)
  {
    std::string from_process, to_process;
    sfl::splitstring(connection_spec, ':', from_process, to_process);
    return CreateChannel(from_process, to_process);
  }
  
  
  wbcnet::Channel * SPQNetConfig::
  CreateChannel(process_t from_process,
		process_t to_process) const
      throw(std::runtime_error)
  {
    return CreateChannel("process__" + sfl::to_string(from_process),
			 "process__" + sfl::to_string(to_process));
  }
  
  
  wbcnet::SPQueue * SPQNetConfig::
  CreateSPQueue(std::string const & from_process,
		std::string const & to_process) const
  {
    LOG_TRACE (logger,
	       "wbcnet::SPQNetConfig::CreateSPQueue(" << from_process << ", "
	       << to_process << ")");
    wbcnet::SPQueue * spq(new wbcnet::SPQueue());
    m_from_to[from_process][to_process] = spq;
    return spq;
  }
  
  
  wbcnet::SPQueue * SPQNetConfig::
  GetSPQueue(std::string const & from_process,
	     std::string const & to_process) const
  {
    from_to_t::const_iterator ifrom(m_from_to.find(from_process));
    if (m_from_to.end() == ifrom)
      return 0;
    to_t::const_iterator ito(ifrom->second.find(to_process));
    if (ifrom->second.end() == ito)
      return 0;
    return ito->second;
  }
  
}
