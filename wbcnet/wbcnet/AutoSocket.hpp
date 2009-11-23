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

/** \file AutoSocket.hpp Wraps around sockets and robustly reconnects or re-listens. */

#ifndef WBCNET_AUTO_SOCKET_HPP
#define WBCNET_AUTO_SOCKET_HPP

#ifdef DISABLE_NETWORKING
# error Networking is DISABLED, do not use this header.
#endif // DISABLE_NETWORKING

#include <wbcnet/SockWrap.hpp>

namespace wbcnet {
  
  
  class AutoSocket
  {
  public:
    AutoSocket(int bufsize,
	       int max_bufsize,
	       bool server_mode,
	       in_port_t port,
	       bool nonblocking,
	       std::string const & address,
	       int domain = AF_INET,
	       int type = SOCK_STREAM,
	       int protocol = 0);
    ~AutoSocket();
    
    int GetCommunicationSocket();
    
    //     com_status Send(BufferAPI const & buffer);
    //     com_status Receive(BufferAPI & buffer);
    
  protected:
    SockWrap * m_sockwrap;
    SoClient * m_client;
    SoServer * m_server;
    in_port_t const m_port;
    bool const m_nonblocking;
    std::string const m_address;
    int const m_domain;
    int const m_type;
    int const m_protocol;
  };
  
}

#endif // WBCNET_AUTO_SOCKET_HPP
