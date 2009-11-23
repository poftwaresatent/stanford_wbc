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

#include "MQWrap.hpp"
#include "data.hpp"
#include <wbcnet/log.hpp>
#include <errno.h>
#include <string.h>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

namespace wbcnet {
  
  
  MQWrap::
  MQWrap(bool auto_close, bool auto_unlink)
    : m_auto_close(auto_close),
      m_auto_unlink(auto_unlink),
      m_queue(-1),
      m_msgsize(-1)
  {
  }
  
  
  MQWrap::
  ~MQWrap()
  {
    if (m_auto_close)
      Close();
    if (m_auto_unlink)
      Unlink();
  }
  
  
  bool MQWrap::
  Open(std::string const & name, mode_t mode,
       long maxmsg, long msgsize,
       bool nonblock)
  {
    if (-1 != m_queue)
      Close();
    m_name = "";
    
    int oflag(O_CREAT);
    if (nonblock)
      oflag |= O_NONBLOCK;
    
    switch (mode) {
    case READ_ONLY:
      oflag |= O_RDONLY;
      break;
    case READ_WRITE:
      oflag |= O_RDWR;
      break;
    case WRITE_ONLY:
      oflag |= O_WRONLY;
      break;
    default:
      LOG_ERROR (logger,
		     "wbcnet::MQWrap::Open(): invalid mode " << mode
		     <<  " (use READ_ONLY, READ_WRITE, WRITE_ONLY)");
      return false;
    };
    
    mq_attr attr;    
    attr.mq_curmsgs = 0;	// probably ignored anyway
    if (nonblock)
      attr.mq_flags = O_NONBLOCK; // probably ignored anyway
    attr.mq_msgsize = msgsize;
    attr.mq_maxmsg = maxmsg;
    
    m_queue = mq_open(name.c_str(), oflag, S_IRUSR | S_IWUSR, &attr);
    
    if (-1 == m_queue) {
      LOG_ERROR (logger,
		     "wbcnet::MQWrap::Open(): mq_open(" << name << "): " << strerror(errno));
      return false;
    }
    
    m_name = name;
    m_msgsize = msgsize;
    return true;
  }
  
  
  void MQWrap::
  Close()
  {
    if (-1 == m_queue)
      return;
    if (0 != mq_close(m_queue))
      LOG_WARN (logger, "wbcnet::MQWrap::Close(): " << strerror(errno));
    m_queue = -1;
  }


  bool MQWrap::
  Unlink()
  {
    if (m_name.empty())
      return true;
    bool ok(true);
    if (0 != mq_unlink(m_name.c_str())) {
      ok = false;
      LOG_ERROR (logger, "wbcnet::MQWrap::Unlink(" << m_name << "): " << strerror(errno));
    }
    m_name = "";
    return ok;
  }
  
  
  com_status MQWrap::
  Send(BufferAPI const & buffer)
  {
    if (-1 == m_queue)
      return COM_NOT_CONNECTED;
    if (0 == mq_send(m_queue, buffer.GetData(), buffer.GetSize(), 0))
      return COM_OK;
    if (EAGAIN == errno)
      return COM_TRY_AGAIN;
    LOG_ERROR (logger, "wbcnet::MQWrap::Send(): mq_send(): " << strerror(errno));
    switch (errno) {
    case EBADF: // The descriptor was invalid.
      return COM_NOT_CONNECTED;
    case EMSGSIZE: // msg_len was greater than the mq_msgsize attribute
      return COM_SIZE_MISMATCH;
    case EINTR: // The call was interrupted by a signal handler.
      return COM_INTERRUPTED;
    case ETIMEDOUT: // The call timed out before a message could be transferred.
      return COM_TIMEOUT;
    }
    return COM_OTHER_ERROR;
  }
  
  
  com_status MQWrap::
  Receive(BufferAPI & buffer)
  {
    if (-1 == m_queue)
      return COM_NOT_CONNECTED;
    if ( ! buffer.Resize(m_msgsize))
      return COM_SIZE_MISMATCH;
    if (-1 != mq_receive(m_queue, buffer.GetData(), buffer.GetSize(), 0))
      return COM_OK;
    if (EAGAIN == errno)
      return COM_TRY_AGAIN;
    LOG_ERROR (logger, "wbcnet::MQWrap::Receive(): mq_receive(): " << strerror(errno));
    switch (errno) {
    case EBADF: // The descriptor was invalid.
      return COM_NOT_CONNECTED;
    case EMSGSIZE: // msg_len was greater than the mq_msgsize attribute
      return COM_SIZE_MISMATCH;
    case EINTR: // The call was interrupted by a signal handler.
      return COM_INTERRUPTED;
    case ETIMEDOUT: // The call timed out before a message could be transferred.
      return COM_TIMEOUT;
    }
    return COM_OTHER_ERROR;
  }
  
}
