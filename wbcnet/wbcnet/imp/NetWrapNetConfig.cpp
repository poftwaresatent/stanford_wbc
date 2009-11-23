/* 
 * Copyright (C) 2009 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#ifdef HAVE_NETWRAP

#include "NetWrapNetConfig.hpp"
#include <wbcnet/NetWrapperWrap.hpp>
#include <wbcnet/strutil.hpp>

#ifndef WIN32
# include <netinet/in.h>
#else
# include "../win32_compat.hpp"
#endif

// #include <wbcnet/log.hpp>

using namespace std;

//static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  NetWrapNetConfig::
  NetWrapNetConfig(bool _server_mode,
		   std::string const & _address)
    : NetConfig(),
      server_mode(_server_mode),
      address(_address)
  {
  }
  
  
  static in_port_t get_tcp_port(NetConfig::process_t from_process,
				NetConfig::process_t to_process)
    throw(runtime_error)
  {
    switch (from_process) {
      
    case NetConfig::SERVO:
      if (NetConfig::MODEL == to_process)
	return 9876;
      if (NetConfig::USER == to_process)
	return 6789;
      throw runtime_error("get_tcp_port(SERVO, " + sfl::to_string(to_process)
			  + "): invalid to_process");
      
    case NetConfig::MODEL:
      if (NetConfig::SERVO == to_process)
	return 9876;
      throw runtime_error("get_tcp_port(MODEL, " + sfl::to_string(to_process)
			  + "): invalid to_process");
      
    case NetConfig::USER:
      if (NetConfig::SERVO == to_process)
	return 6789;
      throw runtime_error("get_tcp_port(USER, " + sfl::to_string(to_process)
			  + "): invalid to_process");
      
      ////case NetConfig::DISTANCE:
      ////case NetConfig::MOTOR:
    }
    
    throw runtime_error("get_tcp_port(" + sfl::to_string(from_process)
			+ ", " + sfl::to_string(to_process)
			+ "): invalid from_process / to_process combination");
    return 0;
  }
  
  
  wbcnet::Channel * NetWrapNetConfig::
  CreateChannel(process_t from_process,
		process_t to_process) const
    throw(runtime_error)
  {

//#ifndef HAVE_NETWRAP
//
//    throw runtime_error("wbcnet::NetWrapNetConfig::CreateChannel(): no support for libnetwrapper");
//    return 0;
//
//#else // HAVE_NETWRAP

    in_port_t const port(get_tcp_port(from_process, to_process));
    TCPNetWrapper * channel(new TCPNetWrapper());
    if ( ! channel->Open(port, address, server_mode, 250000)) {
      delete channel;
      ostringstream os;
      os << "wbcnet::NetWrapNetConfig::CreateChannel(): TCPNetWrapper::Open() failed.\n"
	 << "  from_process: " << from_process << "\n"
	 << "  to_process: " << to_process << "\n"
	 << "  port: " << (int) port << "\n"
	 << "  address: " << address << "\n"
	 << "  server_mode: " << (server_mode ? "true" : "false");
      throw runtime_error(os.str());
    }
    return channel;

//#endif // HAVE_NETWRAP

  }
  

  wbcnet::Channel * NetWrapNetConfig::
  CreateChannel(std::string const & connection_spec) const
    throw(std::runtime_error)
  {

//#ifndef HAVE_NETWRAP
//
//    throw runtime_error("wbcnet::NetWrapNetConfig::CreateChannel(): no support for libnetwrapper");
//    return 0;
//
//#else // HAVE_NETWRAP

    vector<string> token;
    sfl::tokenize(connection_spec, ':', token);
    string keyword;
    sfl::token_to(token, 0, keyword);
    if ("port" != keyword) {
      throw runtime_error("wbcnet::NetWrapNetConfig::CreateChannel(): the only valid keyword is `port' as in `port:8081'");
    }
    in_port_t port;
    if ( ! sfl::token_to(token, 1, port)) {
      throw runtime_error("wbcnet::NetWrapNetConfig::CreateChannel(): missing or invalid port number in spec `" + connection_spec + "'");
    }

    TCPNetWrapper * channel(new TCPNetWrapper());
    if ( ! channel->Open(port, address, server_mode, 250000)) {
      delete channel;
      ostringstream os;
      os << "wbcnet::NetWrapNetConfig::CreateChannel(): TCPNetWrapper::Open() failed.\n"
      << "  connection_spec: " << connection_spec << "\n"
      << "  port: " << (int) port << "\n"
      << "  address: " << address << "\n"
      << "  server_mode: " << (server_mode ? "true" : "false");
      throw runtime_error(os.str());
    }
    return channel;

//#endif // HAVE_NETWRAP

  }

}

#endif // HAVE_NETWRAP
