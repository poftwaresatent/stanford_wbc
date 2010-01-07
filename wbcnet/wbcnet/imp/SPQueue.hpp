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
   \file SPQueue.hpp Queue-based "communication" for single-process applications.
   
   This file contains SPQueue, which is mainly useful for testing
   purposes. Or maybe it can be used as a communication channel
   between threads in a single-process program, but then someone would
   have to take care of proper mutexing.
*/

#ifndef WBCNET_SPQUEUE_HPP
#define WBCNET_SPQUEUE_HPP

#include <wbcnet/com.hpp>
#include <deque>


namespace wbcnet {
  
  
  class Buffer;
  
  
  /**
     Simple std::deque<> based "communication". Sending is simulated
     as appending to the queue, and receiving pops from the front of
     the queue. Very useful for unit tests, probably not much more.
  */
  class SPQueue
    : public Channel
  {
  public:
    virtual ~SPQueue();
    
    virtual com_status Send(BufferAPI const & buffer);
    virtual com_status Receive(BufferAPI & buffer);
    
    bool Empty() const { return m_queue.empty(); }
    size_t GetSize() const { return m_queue.size(); }
    
  protected:
    typedef std::deque<Buffer*> queue_t;
    queue_t m_queue;
  };
  
}

#endif // WBCNET_SPQUEUE_HPP
