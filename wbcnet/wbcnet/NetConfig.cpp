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

#include "NetConfig.hpp"
#include <wbcnet/imp/SPQueue.hpp>
#include <wbcnet/strutil.hpp>
#include <wbcnet/log.hpp>
#include <vector>

#include "imp/SPQNetConfig.hpp"

#ifndef DISABLE_NETWORKING
# include "imp/TCPNetConfig.hpp"
#endif // DISABLE_NETWORKING

#ifdef WBCNET_HAVE_MQUEUE
# include "imp/MQNetConfig.hpp"
# include <stdlib.h>
#endif // WBCNET_HAVE_MQUEUE

#ifdef HAVE_NETWRAP
# include "imp/NetWrapNetConfig.hpp"
#endif // HAVE_NETWRAP


using namespace std;

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  NetConfig * NetConfig::
  Create(std::string const & com) throw(std::runtime_error)
  {
    string name, spec;
    sfl::splitstring(com, ':', name, spec);
    
    NetConfig * cfg(0);
    
    if ("MQ" == name || "mq" == name) {
#ifndef WBCNET_HAVE_MQUEUE
      throw runtime_error("wbcnet::NetConfig::Create(" + com + "): no POSIX message queue support");
#else // WBCNET_HAVE_MQUEUE
      vector<string> token;
      sfl::tokenize(spec, ':', token);
      bool blocking(false);
      if (token.size() > 1) {
	if ("b" == token[0]) {
	  blocking = true;
	}
	else if (("n" == token[0]) || ("" == token[0])) {
	  blocking = false;
	}
	else {
	  throw runtime_error("wbcnet::NetConfig::Create(" + com + "): invalid mode \"" + token[0]
			      + "\" (use 'b', 'n', or '')");
	}
      }
      string prefix;
      if ( ! sfl::token_to(token, 1, prefix)) {
	// no prefix specified, try some environment variables, or
	// fallback to some hardcoded value
	char const * username(getenv("USER"));
	if (0 == username)
	  username = getenv("USERNAME");
	if (0 == username)
	  prefix = "message_queue";
	else
	  prefix = username;
      }
      size_t msg_size;
      if ( ! sfl::token_to(token, 2, msg_size))
	msg_size = 2048;
      LOG_INFO (logger, "wbcnet::NetConfig::Create(): creating MQNetConfig("
		<< sfl::to_string(blocking) << ", " << msg_size << ", " << prefix << ")");
      cfg = new MQNetConfig(blocking, msg_size, prefix);
#endif // WBCNET_HAVE_MQUEUE
    }
    
    else if ("TS" == name || "ts" == name) {
#ifdef DISABLE_NETWORKING
      throw runtime_error("wbcnet::NetConfig::Create(" + com + "): no TCP/IP support");
#else // DISABLE_NETWORKING
      cfg = new TCPServerNetConfig(spec.empty() ? "*" : spec);
#endif // DISABLE_NETWORKING
    }
    
    else if ("TC" == name || "tc" == name) {
#ifdef DISABLE_NETWORKING
      throw runtime_error("wbcnet::NetConfig::Create(" + com + "): no TCP/IP support");
#else // DISABLE_NETWORKING
      cfg = new TCPClientNetConfig(spec.empty() ? "127.0.0.1" : spec);
#endif // DISABLE_NETWORKING
    }
    
    else if ("SP" == name || "sp" == name) {
      cfg = new SPQNetConfig();
    }
    
    else if ("NW" == name || "nw" == name) {
#ifndef HAVE_NETWRAP
      throw runtime_error("wbcnet::NetConfig::Create(" + com + "): no libnetwrapper support");
#else // HAVE_NETWRAP
      vector<string> token;
      sfl::tokenize(spec, ':', token);
      string mode_str;
      sfl::token_to(token, 0, mode_str);
      bool server_mode;
      string address;
      if (mode_str.empty() || mode_str == "s" || mode_str == "S")
	server_mode = true;
      else if (mode_str == "c" || mode_str == "C") {
	server_mode = false;
	if ( ! sfl::token_to(token, 1, address))
	  address = "localhost";
      }
      cfg = new NetWrapNetConfig(server_mode, address);
#endif // HAVE_NETWRAP
    }
    
    if (0 == cfg) {
      ostringstream msg;
      msg << "NetConfig::Create(): invalid com \"" << com << "\"\n"
	  << "Available communication methods are:\n";
      Help("  ", msg);
      throw runtime_error(msg.str());
    }
    
    return cfg;
  }


  void NetConfig::
  Help(std::string const & prefix, std::ostream & os)
  {
#ifdef WBCNET_HAVE_MQUEUE
    os << prefix << "mq [ : prefix [ : msg_size ]]\n"
       << prefix << "   use POSIX message queues\n"
       << prefix << "   defaults: prefix=message_queue msg_size=2048\n";
#endif // WBCNET_HAVE_MQUEUE

#ifndef DISABLE_NETWORKING
    os << prefix << "tc [ : server_ip ]\n"
       << prefix << "   use TCP/IP client mode (connect to a server)\n"
       << prefix << "   default server_ip is * (bind to any local interfaces)\n"
       << prefix << "ts [ : bind_ip ]\n"
       << prefix << "   use TCP/IP server mode (wait for a client to connect)\n"
       << prefix << "   default bind_ip is 127.0.0.1\n";
#endif // DISABLE_NETWORKING

#ifdef HAVE_NETWRAP
    os << prefix << "nw [ : s ] | [ : c [ : address ] ]\n"
       << prefix << "   use libnetwrapper (by default in server mode)\n"
       << prefix << "   say \"nw:c:blah\" to connect as client to server blah\n";
#endif // HAVE_NETWRAP
    
    os << prefix << "sp\n"
       << prefix << "   use single-process 'fake' networking\n";
  }
  

  wbcnet::Channel * NetConfig::
  CreateChannel(std::string const & connection_spec) const
        throw(std::runtime_error)
  {
    throw std::runtime_error("NetConfig::CreateChannel(): Generic version not implemented for this net config");
    return 0;
  }

}
