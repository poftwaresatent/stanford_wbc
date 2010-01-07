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

/** \file NetWrapperWrap.cpp Communication wrapper for the NetWrapper library from Forcedimension. */

#ifdef HAVE_NETWRAP

#include "NetWrapperWrap.hpp"
#include <wbcnet/data.hpp>
#include <wbcnet/log.hpp>
#include <string.h>

#ifdef WIN32
# include "win32_compat.hpp"
#undef SendMessage
#endif

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

using namespace std;

namespace wbcnet {
  
  
  TCPNetWrapper::
  TCPNetWrapper()
    : m_port(0),
      m_address(""),
      m_server_mode(false),
      m_nonblocking(true),
      m_reconnect_usec_sleep(-1),
      m_socket(0)
  {
  }


  TCPNetWrapper::
  ~TCPNetWrapper()
  {
    if (m_server_mode) {
      delete m_socket;
    }
  }
  
  
  bool TCPNetWrapper::
  Open(in_port_t port,
       std::string const & address,
       bool server_mode,
       bool nonblocking,
       long reconnect_usec_sleep)
  {
    if (m_socket) {
      LOG_ERROR (logger, "wbcnet::TCPNetWrapper::Open(): already opened");
      return false;
    }
    
    if (server_mode) {
      if (0 != m_net_wrapper.Listen(port, ! nonblocking)) {
	LOG_ERROR (logger, "wbcnet::TCPNetWrapper::Open(): NetWrapper::TCPSocket::Listen(" << (int) port
		   << ", " << (nonblocking ? "nonblocking" : "blocking") << ") failed");
	return false;
      }
    }	
    
    m_port = port;
    m_address = address;
    m_server_mode = server_mode;
    m_nonblocking = nonblocking;
    m_reconnect_usec_sleep = reconnect_usec_sleep;
    
    return true;
  }
  
  
  bool TCPNetWrapper::
  LazyReconnect(bool silent)
  {
    if (0 != m_socket)
      return true;
    
    if (m_server_mode) {
      if ( ! silent) {
	LOG_INFO (logger,
		  "wbcnet::TCPNetWrapper::LazyReconnect(): server mode: accepting on port "
		  << m_net_wrapper.GetLocalPort());
      }
      m_socket = m_net_wrapper.Accept();
      if (0 == m_socket) {
	if ( ! silent) {
	  LOG_ERROR (logger, "wbcnet::TCPNetWrapper::LazyReconnect(): server mode: connection failed");
	}
	return false;
      }
      if ( ! silent) {
	LOG_INFO (logger,
		  "wbcnet::TCPNetWrapper::LazyReconnect(): server mode: connection from "
		  << m_socket->GetRemoteIP() << " succeeded");
      }
      return true;
    }
    
    if ( ! silent) {
      LOG_INFO (logger,
		"wbcnet::TCPNetWrapper::LazyReconnect(): client mode: connecting to "
		<< m_address << " on port " << (int) m_port
		<< (m_nonblocking ? " in non-blocking mode" : "in blocking mode"));
    }
    if (0 != m_net_wrapper.Connect(m_address.c_str(), m_port, ! m_nonblocking)) {
      if ( ! silent) {
	LOG_ERROR (logger, "wbcnet::TCPNetWrapper::LazyReconnect(): client mode: failed to reconnect to port "
		   << (int) m_port << " on server `" << m_address << "'"
		   << (m_nonblocking ? " in non-blocking mode" : "in blocking mode"));
      }
      return false;
    }
    m_socket = &m_net_wrapper;
    return true;
  }
  
  
  com_status TCPNetWrapper::
  Send(BufferAPI const & buffer)
  {
    if ( ! LazyReconnect(false)) {
      if (0 > m_reconnect_usec_sleep) {
	return COM_NOT_CONNECTED;
      }
      if (0 < m_reconnect_usec_sleep) {
	usleep(m_reconnect_usec_sleep);
      }
      return COM_TRY_AGAIN;
    }
    
    NetWrapper::Message msg;
    msg.PutArray(buffer.GetData(), buffer.GetSize());
    
    if (0 != m_socket->SendMsg(msg)) {
      LOG_ERROR (logger, "wbcnet::TCPNetWrapper::Send(): SendMsg() failed");
      if (0 > m_reconnect_usec_sleep) {
	return COM_OTHER_ERROR;
      }
      if (m_server_mode) {
	delete m_socket;
      }
      m_socket = 0;
      if (0 < m_reconnect_usec_sleep) {
	usleep(m_reconnect_usec_sleep);
      }
      return COM_TRY_AGAIN;
    }
    
    return COM_OK;
  }
  
  
  com_status TCPNetWrapper::
  Receive(BufferAPI & buffer)
  {
    if ( ! LazyReconnect(false)) {
      if (0 > m_reconnect_usec_sleep) {
	return COM_NOT_CONNECTED;
      }
      if (0 < m_reconnect_usec_sleep) {
	usleep(m_reconnect_usec_sleep);
      }
      return COM_TRY_AGAIN;
    }
    
    NetWrapper::Message * msg(m_socket->ReadMsg());
    if (0 == msg) {
      return COM_TRY_AGAIN;
    }
    unsigned int nbytes;
    char const * data(msg->GetArray(&nbytes));
    
    if ( ! buffer.Resize(nbytes)) {
      delete msg;
      LOG_ERROR (logger, "wbcnet::TCPNetWrapper::Receive(): failed to resize buffer");
      return COM_OTHER_ERROR;
    }
    
    memcpy(buffer.GetData(), data, nbytes);
    delete msg;
    return COM_OK;
  }
  
}

#endif // HAVE_NETWRAP
