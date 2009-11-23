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

/** \file NetWrapperWrap.hpp Communication wrapper for the NetWrapper library from Forcedimension. */

#ifndef WBCNET_NETWRAPPER_WRAP_HPP
#define WBCNET_NETWRAPPER_WRAP_HPP

#ifdef HAVE_NETWRAP

#include <wbcnet/com.hpp>
#include <string>

#ifndef WIN32
# include <netinet/in.h>
#else
# include "win32_compat.hpp"
// this undef has to be above NetWrapper.h, because Windows seems to have a preprocessor symbols SendMessage which expands to SendMessageA
# undef SendMessage
#endif

// Forcedimension's NetWrapper.h -- if you don't have it, then don't include this file
// under Windows, make sure to undef SendMessage (see above)
#include <NetWrapper.h>


namespace wbcnet {
  
  
  class TCPNetWrapper
    : public Channel
  {
  public:
    TCPNetWrapper();
    virtual ~TCPNetWrapper();

    bool Open(in_port_t port,
	      std::string const & address,
	      bool server_mode,
	      /** If <0 then do not attempt reconnecting (treat
		  connection failures as errors). Otherwise, after a
		  connection failure the Send() and Receive methods
		  will sleep for the given number of microseconds and
		  then return COM_TRY_AGAIN. If you specify 0
		  microseconds, then these methods immediately return
		  COM_TRY_AGAIN, without forcing a context switch. */
	      long reconnect_usec_sleep);
    
    bool LazyReconnect(bool silent);
    
    virtual com_status Send(BufferAPI const & buffer);
    virtual com_status Receive(BufferAPI & buffer);
    
  protected:
    in_port_t m_port;
    std::string m_address;
    bool m_server_mode;
    long m_reconnect_usec_sleep;
    
    NetWrapper::TCPSocket m_net_wrapper;
    
    // for server sockets this is the client connection
    // for client sockets, this is "the" socket (simply &m_net_wrapper)
    NetWrapper::TCPSocket * m_socket;
  };
  
}

#endif // HAVE_NETWRAP

#endif // WBCNET_NETWRAPPER_WRAP_HPP
