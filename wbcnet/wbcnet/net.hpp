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

/**
   \file net.hpp Basic stream-based networking.

   Using NetSink and NetSource, you can treat any stream-based
   communication channel like a variable-sized message-based channel.
*/

#ifndef WBCNET_NET_HPP
#define WBCNET_NET_HPP

#ifdef DISABLE_NETWORKING
# error Networking is DISABLED, do not use this header.
#endif // DISABLE_NETWORKING


#include <wbcnet/data.hpp>
#include <wbcnet/com.hpp>
#include <stdint.h>


namespace wbcnet {
  

  /**
     Send a stream of packets, hiding partial writes from the
     user. The NetSink has a buffer, and it sends the size of the data
     followed by the buffer contents over the wire (unless the
     skip_length_header flag is set). It sort of mixes the semantics of
     BufferAPI (by composition) and Sink (by similarity of methods).
     
     \note On the receiving end, you can use a NetSource instance to
     do the reverse operation.
  */
  class NetSink
  {
  public:
    typedef enum {
      /** OK to Send() */
      READY,
      /** send in progress: call Send() to get on with it */
      WRITE_LENGTH,
      /** send in progress: call Send() to get on with it */
      WRITE_DATA,
      /** an error occurred in com_writebuf() */
      WRITE_ERROR
    } state;
    
    NetSink(int bufsize, int max_bufsize, bool skip_length_header = false);
    
    /**
       Send the buffer via a file descriptor. The NetSink keeps track
       of partial writes for you, as reflected in
       NetSink::state. Before calling Send(), you probably want to
       prepare NetSink::buffer, which is only safe if the state is
       READY (which is the case if this method returns COM_OK).
       
       \note You are strongly advised to use the same fd for all calls.
    */
    com_status Send(int fd);
    
    /** Use with caution, it discards any ongoing partially completed
	write... the partnering NetSource should be reset, too, if
	that is doable. */
    void Reset();
    
    state GetState() const { return m_state;}
    
    /** \note Only access the buffer if READY==GetState(). */
    Buffer buffer;
    
  protected:
    bool const m_skip_length_header;
    state m_state;
    int32_t m_length;
    char * m_length_wip;
    int m_length_nremain;
    char * m_data_wip;
    int m_data_nremain;
  };
  
  
  /**
     Receive a stream of packets, hiding partial reads from the
     user. The NetSource has a buffer, and it receives the size of the
     data followed by the buffer contents from the wire (unless the
     skip_length_header flag is set). It sort of mixes the semantics of
     BufferAPI (by composition) and Source (by similarity of methods).
     
     \note On the sending end, you can use a NetSink instance to do
     the reverse operation.
  */
  class NetSource
  {
  public:
    typedef enum {
      /** OK to Send(). This is a transitional state, test for DONE
	  before reading from NetSource::buffer. */
      READY,
      /** read in progress: call Receive() to get on with it */
      READ_LENGTH,
      /** Transitional state between READ_LENGTH and READ_DATA, not
	  observable from outside. */
      RESIZE,
      /** read in progress: call Receive() to get on with it */
      READ_DATA,
      /** Read has succeeded, data ready to be used. Automatically
	  transitions to READY the next time or Receive() is
	  called. This state signals to the user that data has arrived
	  and can be used for processing. */
      DONE,
      /** could not resize buffer during Receive() */
      RESIZE_ERROR,
      /** an error occurred in com_writebuf() or com_readbuf() */
      READ_ERROR
    } state;
    
    NetSource(int bufsize, int max_bufsize, bool skip_length_header = false);
    
    /**
       Receive the buffer via a file descriptor. The NetSource keeps
       track of partial reads for you, as reflected in
       NetSource::state. After a successfull Receive(), you probably
       want to retrieve the atomized NetSource::buffer. This is only
       safe if the state is DONE (which is the case if this method
       returns COM_OK).
       
       BEWARE when using skip_length_header=true: the expected length
       is simply taken from the current buffer, so make sure that is
       set to what you expect. Using skip_length_header=true is
       strongly discouraged anyway though.
       
       \note You are strongly advised to use the same fd for all calls.
    */
    com_status Receive(int fd);
    
    /** Use with caution, it discards any ongoing partially completed
	read... the partnering NetSink should be reset, too, if that
	is doable. */
    void Reset();
    
    state GetState() const { return m_state;}
    
    /** \note Only access the buffer if GetState() is DONE. */
    Buffer buffer;
    
  protected:
    bool const m_skip_length_header;
    state m_state;
    int32_t m_length;
    char * m_length_wip;
    int m_length_nremain;
    char * m_data_wip;
    int m_data_nremain;
  };
  
}

#endif // WBCNET_NET_HPP
