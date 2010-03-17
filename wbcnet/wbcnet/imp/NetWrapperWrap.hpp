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

/** \file imp/NetWrapperWrap.hpp Communication wrapper for the NetWrapper library from Forcedimension. */

#ifndef WBCNET_NETWRAPPER_WRAP_HPP
#define WBCNET_NETWRAPPER_WRAP_HPP

#ifndef HAVE_NETWRAP
# error Do not include this header unless you have libnetwrapper.
#endif // HAVE_NETWRAP

#include <wbcnet/com.hpp>
#include <string>

#ifndef WIN32
# include <netinet/in.h>
#else
# include <wbcnet/win32/win32_compat.hpp>
// This undef has to be above NetWrapper.h, because Windows (or one of
// the headers included via win32/win32_compat.hpp) defines a preprocessor
// symbol "SendMessage" which expands to "SendMessageA" and thus mucks
// with our code.
# undef SendMessage
#endif

// Forcedimension's NetWrapper.h -- if you don't have it, then don't include this file.
// Also, under Windows, make sure to undef SendMessage (see above) before including this.
#include <NetWrapper.h>


namespace wbcnet {
  
  
  /**
     A Channel that wraps around Forcedimension's libnetwrapper. Only
     avaiable if you have libnetwrapper on your system and you told the
     wbcnet build system how to locate it.
  */
  class TCPNetWrapper
    : public Channel
  {
  public:
    TCPNetWrapper();
    virtual ~TCPNetWrapper();

    bool Open(in_port_t port,
	      std::string const & address,
	      bool server_mode,
	      /** Whether to use blocking sockets or not. It might
		  seem weird to set nonblocking to false in order to
		  get blocking, but the default operation of wbcnet is
		  nonblocking... that's what it has been designed for,
		  in order to allow single-threaded processes to
		  handle asynchronous messaging. */
	      bool nonblocking,
	      /** If <0 then do not attempt reconnecting (treat
		  connection failures as errors). Otherwise, after a
		  connection failure the Send() and Receive() methods
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
    bool m_nonblocking;
    long m_reconnect_usec_sleep;
    
    NetWrapper::TCPSocket m_net_wrapper;
    
    // for server sockets this is the client connection
    // for client sockets, this is "the" socket (simply &m_net_wrapper)
    NetWrapper::TCPSocket * m_socket;
  };
  
}

#endif // WBCNET_NETWRAPPER_WRAP_HPP
