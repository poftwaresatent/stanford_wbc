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

#include "StreamBufMgr.hpp"
#include <wbcnet/log.hpp>
#include <limits>


static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace {

  static char const * mode_str[] = {
    "FULLSPEED",
    "TRICKLE",
    "STOP"
  };
  
}

namespace wbcnet {
  
  
  StreamBufMgr::
  StreamBufMgr(size_t fullspeed_msg_skip, size_t fullspeed_msg_ceil,
	       size_t trickle_msg_skip, size_t trickle_msg_ceil,
	       size_t stop_msg_ceil)
    : m_fullspeed_msg_skip(fullspeed_msg_skip),
      m_fullspeed_msg_ceil(fullspeed_msg_ceil),
      m_trickle_msg_skip(trickle_msg_skip),
      m_trickle_msg_ceil(trickle_msg_ceil),
      m_stop_msg_ceil(stop_msg_ceil),
      m_count_out(0),
      m_count_in(0),
      m_count_checked(0)
  {
  }
  
  
  size_t StreamBufMgr::
  ComputeLag() const
  {
    if (m_count_in > m_count_out) // overflow
      return m_count_in + (std::numeric_limits<size_t>::max() - m_count_out) + 1;
    return m_count_out - m_count_in;
  }
  
  
  /**
     \todo Maybe we'll need a hysteresis to avoid too rapid mode
     transitions.
  */
  StreamBufMgr::mode_t StreamBufMgr::
  ComputeMode() const
  {
    size_t const diff(ComputeLag());

    LOG_TRACE (logger,
		   "wbcnet::StreamBufMgr::ComputeMode(): in " << m_count_in
		   << "   out " << m_count_out << "   diff " << diff);
    
    if (diff <= m_fullspeed_msg_ceil)
      return FULLSPEED;
    if (diff <= m_trickle_msg_ceil)
      return TRICKLE;
    return STOP;
  }
  
  
  void StreamBufMgr::
  IncOutgoing()
  {
    ++m_count_out;
  }
  
  
  void StreamBufMgr::
  IncIncoming()
  {
    ++m_count_in;
  }
  
  
  bool StreamBufMgr::
  CheckSend(mode_t * opt_mode)
  {
    ++m_count_checked;
    mode_t const mode(ComputeMode());
    if (opt_mode)
      *opt_mode = mode;
    bool result(false);
    if (FULLSPEED == mode)
      result = ((0 == m_fullspeed_msg_skip) || (0 == (m_count_checked % m_fullspeed_msg_skip)));
    else if (TRICKLE == mode)
      result = ((0 == m_trickle_msg_skip) || (0 == (m_count_checked % m_trickle_msg_skip)));
    
    LOG_TRACE (logger,
		   "wbcnet::StreamBufMgr::CheckSend(): " << m_count_checked << " "
		   << mode_str[mode] << (result ? " send" : " wait"));
    
    return result;
  }
  
}
