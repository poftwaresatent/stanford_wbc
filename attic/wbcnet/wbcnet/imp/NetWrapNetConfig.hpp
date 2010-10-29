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

#ifndef WBCNET_NET_WRAP_NET_CONFIG_HPP
#define WBCNET_NET_WRAP_NET_CONFIG_HPP

#include <wbcnet/NetConfig.hpp>

namespace wbcnet {
  
  class TCPNetWrapper;
  
  
  /**
     A NetConfig that creates a TCPNetWrapper for you. This allows you
     to communicate via Forcedimension's libnetwrapper, if that is
     available on your system.
  */
  class NetWrapNetConfig
    : public NetConfig
  {
  public:
    typedef NetConfig::process_t process_t;
  
    bool server_mode;
    std::string address;
    
    NetWrapNetConfig(/** If true, use a server mode net wrap
			 socket. Otherwise, use client mode. */
		     bool server_mode,
		     /** only used for client mode */
		     std::string const & address);
    
    virtual wbcnet::Channel * CreateChannel(process_t from_process,
					    process_t to_process) const
      throw(std::runtime_error);

    /** For connection specs of the form "port:1234", create a libnetwrapper connection on that port number. */
    virtual wbcnet::Channel * CreateChannel(std::string const & connection_spec) const
              throw(std::runtime_error);
  };
  
}

#endif // WBCNET_NET_WRAP_NET_CONFIG_HPP
