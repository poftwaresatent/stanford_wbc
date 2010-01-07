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

#include "SockWrap.hpp"
#include <wbcnet/log.hpp>
#include <string.h>
#include <strings.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>


static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  SockWrap::
  SockWrap(int bufsize, int max_bufsize,
	   int & com_sockfd,
	   bool skip_length_header)
    : m_nsink(bufsize, max_bufsize, skip_length_header),
      m_nsource(bufsize, max_bufsize, skip_length_header),
      m_sockfd(-1),
      m_com_sockfd(com_sockfd)
  {
  }
  
  
  SockWrap::
  ~SockWrap()
  {
    LOG_TRACE (logger, "wbcnet::SockWrap dtor: calling Close()");
    Close();
  }
  
  
  bool SockWrap::
  Open(in_port_t port,
       bool nonblocking,
       std::string const & address,
       int domain,
       int type,
       int protocol)
  {
    Close();
    
    m_port = port;
    m_address = address;
    m_domain = domain;
    m_type = type;
    m_protocol = protocol;
    
    bzero(&m_name, sizeof(m_name));
    sockaddr_in * in_name((sockaddr_in*) &m_name);
    if ("*" == address)
      in_name->sin_addr.s_addr = INADDR_ANY;
    else {
      switch (inet_aton(address.c_str(), &in_name->sin_addr)) {
      case 1:
	break;
      case 0:
	LOG_ERROR (logger, "wbcnet::SockWrap::Open(): inet_aton() failed to parse" << address);
	return false;
      default:
	LOG_ERROR (logger, "wbcnet::SockWrap::Open(): inet_aton(): " << strerror(errno));
	return false;
      }
    }
    in_name->sin_family = domain;
    in_name->sin_port = htons(port);
    
    m_sockfd = socket(domain, type, protocol);
    if (0 > m_sockfd) {
      LOG_ERROR (logger,
		     "wbcnet::SockWrap::Open(): socket(): " << strerror(errno) << "\n"
		     << "  domain: " << domain << "\n"
		     << "  type: " << type << "\n"
		     << "  protocol: " << protocol);
      return false;
    }
    
    if ((nonblocking) && (-1 == fcntl(m_sockfd, F_SETFL, O_NONBLOCK))) {
      LOG_ERROR (logger, "wbcnet::SockWrap::Open(): fcntl(...O_NONBLOCK): " << strerror(errno));
      return false;
    }
    
    return true;
  }
  
  
  void SockWrap::
  Close()
  {
    LOG_TRACE (logger, "wbcnet::SockWrap::Close()");
    
    if (-1 != m_sockfd) {
      if (-1 == close(m_sockfd))
	LOG_ERROR (logger, "wbcnet::SockWrap::Close(): close(): " << strerror(errno));
      m_sockfd = -1;
    }
    
    // discard any pending data
    m_nsink.Reset();
    m_nsource.Reset();
  }
  
  
  SoClient::
  SoClient(int bufsize, int max_bufsize,
	   bool skip_length_header)
    :  // ref to uninitialized field is OK here
    SockWrap(bufsize, max_bufsize, m_sockfd, skip_length_header),
    m_connected(false)
  {
  }
  
  
  com_status SoClient::
  Connect()
  {
    if (m_connected)
      return COM_OK;
    
    if (-1 == m_sockfd) {
      LOG_ERROR (logger, "wbcnet::SoClient::Connect(): not open");
      return COM_NOT_OPEN;
    }
    
    LOG_TRACE (logger, "wbcnet::SoClient::Connect(): calling connect()");
      
    if (0 != connect(m_sockfd, (sockaddr*) &m_name, sizeof(sockaddr_in))) {
      switch (errno) {
      case EISCONN:
	break;	     // "success" (this happens under OpenBSD, Darwin, and QNX...)
      case EAGAIN:
      case EINPROGRESS:
      case EALREADY:
	return COM_TRY_AGAIN;
      default:
	LOG_ERROR (logger,
		       "wbcnet::SoClient::Connect(): connect(): " << strerror(errno) << "\n"
		       << "  domain: " << m_domain << "\n"
		       << "  address: " << m_address << "\n"
		       << "  port: " << m_port);
	if (EINTR == errno)
	  return COM_INTERRUPTED;
	if (ETIMEDOUT == errno)
	  return COM_TIMEOUT;
	return COM_OTHER_ERROR;
      }
    }
    
    LOG_TRACE (logger, "wbcnet::SoClient::Connect(): connect() succeeded");
    
    m_connected = true;
    return COM_OK;
  }
  
  
  SoServer::
  SoServer(int bufsize, int max_bufsize,
	   bool skip_length_header)
    :  // ref to uninitialized field is OK here
    SockWrap(bufsize, max_bufsize, m_client_sockfd, skip_length_header),
    m_bound(false),
    m_accepted(false),
    m_client_sockfd(-1)
  {
  }
  
  
  bool SoServer::
  BindListen(int backlog)
  {
    if (m_bound)
      return true;
    
    if (-1 == m_sockfd) {
      LOG_ERROR (logger, "wbcnet::SoServer::BindListen(): not open");
      return false;
    }
    
    LOG_TRACE (logger, "wbcnet::SoServer::BindListen(): calling bind()");
    
    if (0 != bind(m_sockfd, (sockaddr*) &m_name, sizeof(sockaddr_in))) {
      LOG_ERROR (logger,
		     "wbcnet::SoServer::BindListen(): bind(): "
		     << strerror(errno) << "\n"
		     << "  domain: " << m_domain << "\n"
		     << "  address: " << m_address << "\n"
		     << "  port: " << m_port);
      return false;
    }
    
    LOG_TRACE (logger, "wbcnet::SoServer::BindListen(): calling listen()");
    
    if (0 != listen(m_sockfd, backlog)) {
      LOG_ERROR (logger,
		     "wbcnet::SoServer::BindListen(): listen(): "
		     << strerror(errno) << "\n"
		     << "  backlog: " << backlog);
      return false;
    }
    
    LOG_TRACE (logger, "wbcnet::SoServer::BindListen(): m_sockfd: " << m_sockfd);
    
    m_bound = true;
    return true;
  }
  
  
  com_status SoServer::
  Accept()
  {
    if (m_accepted)
      return COM_OK;
    
    if (-1 == m_sockfd)
      return COM_NOT_OPEN;
    
    if ( ! m_bound) {
      LOG_ERROR (logger, "wbcnet::SoServer::Accept(): call BindListen() first");
      return COM_NOT_BOUND;
    }
    
    LOG_TRACE (logger, "wbcnet::SoServer::Accept(): calling accept()");
    
    socklen_t len(sizeof(m_client_name));
    m_client_sockfd = accept(m_sockfd, (sockaddr*) &m_client_name, &len);
    if (-1 == m_client_sockfd) {
      if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
	return COM_TRY_AGAIN;
      LOG_ERROR (logger, "wbcnet::SoServer::Accept(): accept(): " << strerror(errno));
      if (EINTR == errno)
	return COM_INTERRUPTED;
      return COM_OTHER_ERROR;
    }
    
    if (logger->isTraceEnabled()) {
      sockaddr_in const * cliname((sockaddr_in*) &m_client_name);
      LOG_TRACE (logger,
		     "wbcnet::SoServer::Accept(): DEBUG m_client_sockfd: "
		     << m_client_sockfd << "\n"
		     << "  client: " << inet_ntoa(cliname->sin_addr) << ":"
		     << ntohs(cliname->sin_port));
    }
    
    m_accepted = true;
    return COM_OK;
  }
  
  
  void SoServer::
  CloseClient()
  {
    LOG_TRACE (logger, "wbcnet::SoServer::CloseClient()");
    
    if (-1 != m_client_sockfd) {
      if (-1 == close(m_client_sockfd))
	LOG_ERROR (logger, "wbcnet::SoServer::CloseClient(): close(): " << strerror(errno));
      m_client_sockfd = -1;
    }
    
    // discard any pending data
    m_nsink.Reset();
    m_nsource.Reset();
  }
  
  
  void SoServer::
  Close()
  {
    LOG_TRACE (logger, "wbcnet::SoServer::Close()");
    
    CloseClient();
    SockWrap::Close();
    if ((AF_UNIX == m_domain) && (-1 == unlink(m_address.c_str())))
      LOG_ERROR (logger,
		     "wbcnet::SoServer::Close(): unlink(): "
		     << strerror(errno) << "\n"
		     << "  address: " << m_address);
  }
  
  
  com_status SockWrap::
  Send(BufferAPI const & buffer)
  {
    if (-1 == m_com_sockfd)
      return COM_NOT_CONNECTED;
    
    if (NetSink::READY != m_nsink.GetState()) {
      if (NetSink::WRITE_ERROR == m_nsink.GetState()) {
	LOG_ERROR (logger,
		       "wbcnet::SockWrap::Send(): m_nsink is in NetSink::WRITE_ERROR state");
	return COM_OTHER_ERROR;
      }
      
      //////////////////////////////////////////////////
      // Oops, I think there's a bug here, which is hidden by the fact
      // that most of the time, writing will succeed in its entirety
      // because the kernel just copies it to the outgoing
      // buffer... if the state is neither READY nor WRITE_ERROR, we
      // should Send() again without setting the buffer, so that the
      // old one keeps being sent, and in case of success do whatever
      // is done after this if statement.
      //
      // e.g.:
      // if (NetSink::COM_OK != m_nsink.Send(m_com_sockfd)) {
      // 	LOG_TRACE (logger, "wbcnet::SockWrap::Send(): m_nsink is not NetSink::READY (yet)");
      // 	return COM_TRY_AGAIN;
      // }
      // else set the bufer etc...
      //
      // See also Receive(), which DOES receive each time unless there
      // is an error.
      //////////////////////////////////////////////////
      
      LOG_TRACE (logger, "wbcnet::SockWrap::Send(): m_nsink is not NetSink::READY (yet)");
      return COM_TRY_AGAIN;
    }
    
    if ( ! m_nsink.buffer.Set(buffer)) {
      LOG_ERROR (logger, "wbcnet::SockWrap::Send(): m_nsink.buffer.Set(buffer) failed");
      return COM_OTHER_ERROR;
    }
    
    LOG_TRACE (logger, "wbcnet::SockWrap::Send(): m_com_sockfd: " << m_com_sockfd);
    
    return m_nsink.Send(m_com_sockfd);
  }
  
  
  com_status SockWrap::
  Receive(BufferAPI & buffer)
  {
    if (-1 == m_com_sockfd)
      return COM_NOT_CONNECTED;
    
    LOG_TRACE (logger, "wbcnet::SockWrap::Receive(): m_com_sockfd: " << m_com_sockfd);
    
    com_status const cs(m_nsource.Receive(m_com_sockfd));
    
    if (COM_TRY_AGAIN == cs)
      return COM_TRY_AGAIN;
    
    if (COM_OK != cs) {
      LOG_ERROR (logger,
		     "wbcnet::SockWrap::Receive(): m_nsource.Receive() failed with "
		     << com_status_str(cs));
      LOG_TRACE (logger,
		     "wbcnet::SockWrap::Receive(): m_nsource.GetState(): " << m_nsource.GetState());
      return COM_OTHER_ERROR;
    }
    
    if ( ! buffer.Set(m_nsource.buffer)) {
      LOG_ERROR (logger, "wbcnet::SockWrap::Receive(): buffer.Set(m_nsource.buffer) failed");
      return COM_OTHER_ERROR;
    }
    
    return COM_OK;
  }
  
}
