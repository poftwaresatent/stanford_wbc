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

/** \file imp/SockWrap.hpp Communication interface that wraps around sockets. */

#ifndef WBCNET_SOCKWRAP_HPP
#define WBCNET_SOCKWRAP_HPP

#ifdef DISABLE_NETWORKING
# error Networking is DISABLED, do not use this header.
#endif // DISABLE_NETWORKING

#include <wbcnet/com.hpp>
#include <wbcnet/net.hpp>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string>


namespace wbcnet {
  
  
  /**
     A communication interface that wraps a socket. SockWrap can
     represent a client or server using the specialized subclasses
     SoSclient and SoServer.
     
     \note The interface "should" be generic enough to handle any kind
     of socket, but so far only TCP over IP version 4 has been tested.
  */
  class SockWrap
    : public Channel
  {
  protected:
    SockWrap(int bufsize, int max_bufsize,
	     /** Subclasses pass in the sockfd which is to be used in
		 Send() and Receive(). */
	     int & com_sockfd,
	     /** Gets passed to NetSink and NetSource... set it to
		 true only if you really know what you're doing. */
	     bool skip_length_header = false);
    
  public:
    /** \note Calls Close(). */
    virtual ~SockWrap();
    
    /**
       Create a socket. Basically wraps the socket() system
       call. Calls Close() in case we are already open.
       
       \return true if everything succeeded.
    */
    bool Open(in_port_t port,
	      bool nonblocking, // = true,
	      std::string const & address, // = "127.0.0.1",
	      int domain = AF_INET,
	      int type = SOCK_STREAM,
	      int protocol = 0);
    
    bool Opened() const { return -1 != m_sockfd; }
    int GetSocket() const { return m_sockfd; }
    
    virtual void Close();
    
    /**
       Send some data via a socket. Performs some checks, such as
       verifying that Open() has been called before, then delegates
       the actual work to NetSink::Send().
       
       \return COM_OK if everything went according to plan. Note that
       COM_TRY_AGAIN is fairly common when you are using non-blocking
       I/O, which is the default.
    */
    com_status Send(BufferAPI const & buffer);
    
    /**
       The counterpart of Send(). Likewise, uses NetSource::Receive()
       for the real work.
       
       \note When using skip_length_header=true (which presumes that
       you know what you're doing) it can be tricky to make sure that
       the right number of bytes is expected. Use ResizeSourceBuffer()
       for that.
       
       \return COM_OK if everything went according to plan. Note that
       COM_TRY_AGAIN is fairly common when you are using non-blocking
       I/O, which is the default.
    */
    com_status Receive(BufferAPI & buffer);
    
    /**
       Only really makes sense When using skip_length_header=true, it
       can be useful before a call to Receive()... use at your own
       risk though.
     */
    bool ResizeSourceBuffer(int size);
    
  protected:
    NetSink m_nsink;
    NetSource m_nsource;
    in_port_t m_port;
    std::string m_address;
    int m_domain;
    int m_type;
    int m_protocol;
    struct sockaddr_storage m_name;
    int m_sockfd;
    int & m_com_sockfd;
  };
  
  
  /**
     A client socket interface. How to use SoClient:
     - Open() once
     - Connect() once
     - Send() and Receive() at will
     - Close()... or just let the destructor handle that.
     
     Possibly reusable by calling Close() and then Open() etc
     again... but this has not been tested.
  */
  class SoClient
    : public SockWrap
  {
  public:
    /**
       Create a client socket. All parameters are passed to the
       SockWrap constructor. The bufsize and max_bufsize parameters
       end up in NetSink and NetSource and serve to define their
       buffer behavior, just like size and max_size parameters of the
       Buffer constructor.
     */
    SoClient(int bufsize, int max_bufsize,
	     /** Set it to true only if you really know what you're
		 doing. */
	     bool skip_length_header = false);
    
    /**
       Connect to a server, typically a SoServer instance. Has to be
       called before Send() and Receive(). And obviously (?)  you have
       to call Open() before Connect().
    */
    com_status Connect();
    
    bool Connected() const { return m_connected; }
    
  protected:
    /** \todo This field should be reset in Close() or on other errors. */
    bool m_connected;
  };
  
  
  /**
     A server socket interface. How to use a SoServer:
     - Open() once
     - BindListen() once
     - Accept() once per client connection (possibly once overall)
     - maybe CloseClient() in order to accept a new client
     - maybe Close() to reuse the whole thing...
     
     Possibly reusable by calling Close() and the Open() etc
     again... but this has not been tested.
  */
  class SoServer
    : public SockWrap
  {
  public:
    /**
       Create a server socket. All parameters are passed to the
       SockWrap constructor. The bufsize and max_bufsize parameters
       end up in NetSink and NetSource and serve to define their
       buffer behavior, just like size and max_size parameters of the
       Buffer constructor.
     */
    SoServer(int bufsize, int max_bufsize,
	     /** Set it to true only if you really know what you're
		 doing. */
	     bool skip_length_header = false);
    
    /**
       Bind to and listen on the port specified during Open().  This
       method has to be called before Accept(), which has to be called
       before Send() and Receive(). And obviously (?) you have to call
       Open() before BindListen().
    */
    bool BindListen(int backlog = 1);
    
    bool Bound() const { return m_bound; }
    
    /**
       Accept an incoming connection, typically from a SoClient.  This
       method has to be called before Send() and Receive(). And
       obviously (?) you have to call BindListen() before Accept().
    */
    com_status Accept();
    
    bool Accepted() const { return m_accepted; }
    
    int GetClientSocket() const { return m_client_sockfd; }
    
    void CloseClient();
    virtual void Close();
    
  protected:
    /** \todo This field should be reset in Close() or on other errors. */
    bool m_bound;
    /** \todo This field should be reset in Close() or on other errors. */
    bool m_accepted;
    struct sockaddr_storage m_client_name;
    int m_client_sockfd;
  };
  
}

#endif // WBCNET_SOCKWRAP_HPP
