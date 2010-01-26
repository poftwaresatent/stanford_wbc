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

/** \file StreamBufMgr.hpp Helps you optimize delay and frequency over the network. */

#ifndef WBCNET_STREAM_BUF_MGR_HPP
#define WBCNET_STREAM_BUF_MGR_HPP

#include <cstddef>
#include <iosfwd>

namespace wbcnet {
  

  /**
     Utility for achieving "just in time" communication flow. This is
     useful when you do not know the exact delays and number of
     buffers along the way from source to destination. It uses a
     Kanban production model to decide when to send messages full
     blast, when to trickle them into the stream, and when to stop
     sending altogether. The aim is to reach a state where we send at
     regular intervals without flooding or starving any buffer in the
     loop.
  */
  class StreamBufMgr
  {
  public:
    typedef enum {
      FULLSPEED,
      TRICKLE,
      STOP
    } mode_t;
    
    StreamBufMgr(/** How many times to skip in CheckSend() before
		     returning true, in case the mode is
		     FULLSPEED. You should probably set this value to
		     0 (or 1, which is treated identically). */
		 size_t fullspeed_msg_skip,
		 /** The largest allowed difference between the
		     outgoing and incoming message counters for
		     FULLSPEED mode. If the difference becomes larger
		     than this, we go to mode TRICKLE or STOP. */
		 size_t fullspeed_msg_ceil,
		 /** How many times to skip in CheckSend() before
		     returning true when we are in TRICKLE mode. This
		     allows to slowly keep feeding the network even if
		     trickle_msg_ceil has been exceeded. */
		 size_t trickle_msg_skip,
		 /** The message count difference up to which we
		     remain in TRICKLE mode, which makes CheckSend()
		     return true every trickle_msg_skip times you call
		     it. */
		 size_t trickle_msg_ceil,
		 /** The largest allowed difference between outgoing
		     and incoming counters. If the difference is
		     larger than that, we will STOP and CheckSend()
		     always returns false (until the incoming counter
		     has caught up sufficiently with the outgoing
		     counter).
		 */
		 size_t stop_msg_ceil);
    
    /**
       \return The number of packets currently under way (in buffers
       or processing).
    */
    size_t ComputeLag() const;
    
    /**
       \return The mode we are in, based on the difference between the
       outgoing and incoming message counters and the parameters
       fullspeed_msg_ceil, trickle_msg_ceil, and stop_msg_ceil.
    */
    mode_t ComputeMode() const;
    
    /**
       Call this each time after you have successfully sent a message.
    */
    void IncOutgoing();
    
    /**
       Call this each time after you have successfully received a message.
    */
    void IncIncoming();
    
    /**
       Call this each time before you WOULD send a message. If it
       returns true, then go ahead and send it (and then call
       IncOutgoing() of course). If it returns false, then DO NOT send
       the message, simply skip it this time (and abviously
       IncOutgoing() must not be called either).
    */
    bool CheckSend(/** if non-null, the current mode will be returned
		       in here */
		   mode_t * opt_mode);
    
  protected:
    size_t const m_fullspeed_msg_skip;
    size_t const m_fullspeed_msg_ceil;
    size_t const m_trickle_msg_skip;
    size_t const m_trickle_msg_ceil;
    size_t const m_stop_msg_ceil;
    size_t m_count_out;
    size_t m_count_in;
    size_t m_count_checked;
  };
  
}

#endif // WBCNET_STREAM_BUF_MGR_HPP
