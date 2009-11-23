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

/** \file com.hpp Basic communication abstractions and utilities. */

#ifndef WBCNET_COM_HPP
#define WBCNET_COM_HPP


#include <iosfwd>


namespace wbcnet {
  
  
  class BufferAPI;
  
  
  /**
     Return values of many communication-related functions and
     methods. You can use com_status_str() to retrieve statically
     allocated strings containing the symbolic name of a given
     com_status.
  */
  typedef enum {
    COM_OK,
    COM_NOT_OPEN,
    COM_NOT_CONNECTED,
    COM_NOT_BOUND,
    COM_TRY_AGAIN,
    COM_SIZE_MISMATCH,
    COM_INTERRUPTED,
    COM_TIMEOUT,
    COM_OTHER_ERROR
  } com_status;
  
  /**
     \return A static string containing the symbolic name of a
     com_status. Very useful for producing human-readable error
     messages.
  */
  char const * com_status_str(com_status cs);
  
#ifndef WIN32
  // we do not need these under windows anyway

  /**
     Utility for writing a buffer into a file. Uses the system's
     write() routine to send some data using a file descriptor,
     looping until either everything has been written or an error
     occurred. Note that we are most interested in the system errno
     being EAGAIN, which happens a lot in non-blocking
     I/O. NetSink::Send() is the main (only?) user of this function,
     but maybe it will be handy for other users as well.
  
     \return the number of bytes that remain to be written (i.e. 0 on
     success) or -1 on error, in which case errno reflects what went
     wrong with the underlying write() call.
  */
  int com_writebuf(int fd, char const * data, int n_bytes,
		   /** optional */
		   int * n_written);
  
  /**
     Utility for reading a file into a buffer. Uses the system's
     read() routine to receive some data using a file descriptor,
     looping until either everything has been written or an error
     occurred. Note that we are most interested in the system errno
     being EAGAIN, which happens a lot in non-blocking
     I/O. NetSource::Receive() is the main (only?) user of this
     function, but maybe it will be handy for other users as well.
     
     \return the number of bytes that remain to be read (i.e. 0 on
     success) or -1 on error, in which case errno reflects what went
     wrong with the underlying read() call.
  */
  int com_readbuf(int fd, char * data, int n_bytes,
		  /** optional */
		  int * n_read);

#endif // WIN32

  
  /**
     Requirement for things that can send.
  */
  class Sink {
  public:
    virtual ~Sink() {}
    
    /**
       Send a buffer's contents.
       
       \return COM_OK if it worked, or one of the other com_status
       codes if it failed.
    */
    virtual com_status Send(BufferAPI const & buffer) = 0;
  };
  
  
  /**
     Requirement for things that can receive.
  */
  class Source {
  public:
    virtual ~Source() {}
    
    /**
       Receive data and store it in a buffer.
       
       \return COM_OK if it worked, or one of the other com_status
       codes if it failed.
    */
    virtual com_status Receive(BufferAPI & buffer) = 0;
  };
  
  
  /**
     Requirement for things that can send and receive.
  */
  class Channel
    : public Sink,
      public Source
  {
  };
  
  
  /**
     A Channel that wraps an existing Sink and Source.
  */
  class ProxyChannel
    : public Channel
  {
  public:
    ProxyChannel(Sink * sink,
		 /** if true, then sink will be deleted in ~ProxyChannel() */
		 bool own_sink,
		 Source * source,
		 /** if true, then source will be deleted in ~ProxyChannel() */
		 bool own_source);
    
    virtual ~ProxyChannel();
    
    virtual com_status Send(BufferAPI const & buffer);
    virtual com_status Receive(BufferAPI & buffer);
    
  private:
    Sink * m_sink;
    bool const m_own_sink;
    Source * m_source;
    bool m_own_source;
  };
  
}

#endif // WBCNET_COM_HPP
