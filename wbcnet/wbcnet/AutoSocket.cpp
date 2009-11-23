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

#include "AutoSocket.hpp"
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  AutoSocket::
  AutoSocket(int bufsize,
	     int max_bufsize,
	     bool server_mode,
	     in_port_t port,
	     bool nonblocking,
	     std::string const & address,
	     int domain,
	     int type,
	     int protocol)
    : m_sockwrap(0),
      m_client(0),
      m_server(0),
      m_port(port),
      m_nonblocking(nonblocking),
      m_address(address),
      m_domain(domain),
      m_type(type),
      m_protocol(protocol)
  {
    if (server_mode) {
      m_server = new SoServer(bufsize, max_bufsize);
      m_sockwrap = m_server;
    }
    else {
      m_client = new SoClient(bufsize, max_bufsize);
      m_sockwrap = m_client;
    }
  }
  
  
  AutoSocket::
  ~AutoSocket()
  {
    delete m_sockwrap;
  }
  
  
  int AutoSocket::
  GetCommunicationSocket()
  {
    if ( ! m_sockwrap->Opened()) {
      LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_sockwrap->Open()");
      if ( ! m_sockwrap->Open(m_port, m_nonblocking, m_address, m_domain, m_type, m_protocol)) {
	LOG_ERROR (logger, "AutoSocket::GetCommunicationSocket(): m_sockwrap->Open() failed");
	return -1;
      }
      LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_sockwrap->Open() OK");
    }
    
    if (m_client) {
      if ( ! m_client->Connected()) {
	LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_client->Connect()");
	if (COM_OK != m_client->Connect()) {
	  LOG_ERROR (logger,
			 "AutoSocket::GetCommunicationSocket(): m_client->Connect() failed");
	  return -1;
	}
	LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_client->Connect() OK");
      }
      LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_client->GetSocket()");
      return m_client->GetSocket();
    }
    else if (m_server) {
      if ( ! m_server->Bound()) {
	LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_server->BindListen()");
	if ( ! m_server->BindListen()) {
	  LOG_ERROR (logger,
			 "AutoSocket::GetCommunicationSocket(): m_server->BindListen() failed");
	  return -1;
	}
	LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_server->BindListen() OK");
      }
      if ( ! m_server->Accepted()) {
	LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_server->Accept()");
	if (COM_OK != m_server->Accept()) {
	  LOG_ERROR (logger, "AutoSocket::GetCommunicationSocket(): m_server->Accept() failed");
	  return -1;
	}
	LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_server->Accept() OK");
      }
      LOG_TRACE (logger, "AutoSocket::GetCommunicationSocket(): m_server->GetClientSocket()");
      return m_server->GetClientSocket();
    }
    
    LOG_ERROR (logger, "AutoSocket::GetCommunicationSocket(): BUG! neither client nor server");
    return -1;
  }
  
}
