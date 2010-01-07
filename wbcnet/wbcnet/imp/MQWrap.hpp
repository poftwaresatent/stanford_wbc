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

/** \file MQWrap.hpp Communication wrapper for POSIX message queues. */

#ifndef WBCNET_MQWRAP_HPP
#define WBCNET_MQWRAP_HPP

#include <wbcnet/com.hpp>
#include <mqueue.h>
#include <string>


namespace wbcnet {
  
  
  /**
     Communicate using POSIX message queues. This has been tested
     under QNX 6 and Linux 2.6. If your system has a header file
     called mqueue.h you're probably in good shape for using this.
     
     \note Message queues require a constant fixed size for their
     messages, and they cannot send messages bigger than that. They
     would have to be split into several chunks, but then you probably
     lose the advantage of mqueues and might as well use TCP or so.
  */
  class MQWrap
    : public Channel
  {
  public:
    /**
       Direction of this communication endpoint. READ_WRITE only
       really makes sense for testing, when you want to read messages
       that you've sent from the same process.
    */
    typedef enum {
      READ_ONLY,
      READ_WRITE,
      WRITE_ONLY
    } mode_t;
    
    MQWrap(/** Automatically Close() during destruction. This is
	       probably a good idea. */
	   bool auto_close,
	   /** Automatically Unlink() during destruction. This is also
	       recommended, but somehow does not always seem to
	       work. */
	   bool auto_unlink);
    ~MQWrap();
    
    /** Basically just wraps mq_open(). See man mq_overview(7) (on
	Linux) for information about valid maxmsg and msgsize. You can
	probably look at /proc/sys/fs/mqueue/msg_max and
	/proc/sys/fs/mqueue/msgsize_max for upper limits. */
    bool Open(std::string const & name, mode_t mode,
	      long maxmsg, long msgsize,
	      bool nonblock);
    
    /** Basically just wraps mq_close(). */    
    void Close();
    
    /** Basically just wraps mq_unlink(). */    
    bool Unlink();
    
    mqd_t Desc() const { return m_queue; }
    
    virtual com_status Send(BufferAPI const & buffer);
    virtual com_status Receive(BufferAPI & buffer);
    
  protected:
    bool m_auto_close;
    bool m_auto_unlink;
    std::string m_name;
    mqd_t m_queue;
    long m_msgsize;
  };
  
}

#endif // WBCNET_MQWRAP_HPP
