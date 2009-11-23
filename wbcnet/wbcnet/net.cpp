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

#include "net.hpp"
#include <wbcnet/log.hpp>
#include <errno.h>
#include <string.h>
////#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {


  NetSink::
  NetSink(int bufsize, int max_bufsize)
    : buffer(bufsize, max_bufsize)
  {
    Reset();
  }
  
  
  void NetSink::
  Reset()
  {
    m_state = READY;
    
    // the rest is overkill...
    m_length = -1;
    m_length_wip = 0;
    m_length_nremain = 0;
    m_data_wip = 0;
    m_data_nremain = 0;
  }
  
  
  com_status NetSink::
  Send(int fd)
  {
    if (WRITE_ERROR == m_state) {
      LOG_ERROR (logger, "wbcnet::NetSink::Send(): WRITE_ERROR state");
      return COM_OTHER_ERROR;
    }
    
    if (READY == m_state) {
      m_length = buffer.GetSize();
      m_length_wip = (char*) &m_length;
      m_length_nremain = sizeof(m_length);
      m_data_wip = buffer.GetData();
      m_data_nremain = buffer.GetSize();
      m_state = WRITE_LENGTH;
    }
    
    if (WRITE_LENGTH == m_state) {
      int nwritten;
      int const
	nremain(com_writebuf(fd, m_length_wip, m_length_nremain, &nwritten));
      if ((-1 == nremain) && (EAGAIN != errno)) {
	LOG_ERROR (logger,
		       "wbcnet::NetSink::Send(): com_writebuf() failed during WRITE_LENGTH: "
		       << strerror(errno));
	m_state = WRITE_ERROR;
	return COM_OTHER_ERROR;
      }
      
      if (0 == nremain)	// can also be -1 if EAGAIN, will return COM_TRY_AGAIN at end
	m_state = WRITE_DATA;
      else {
	m_length_wip += nwritten;
	m_length_nremain -= nwritten;
      }
    }
    
    if (WRITE_DATA == m_state) {
      int nwritten;
      int const
	nremain(com_writebuf(fd, m_data_wip, m_data_nremain, &nwritten));
      if ((-1 == nremain) && (EAGAIN != errno)) {
	LOG_ERROR (logger,
		       "wbcnet::NetSink::Send(): com_writebuf() failed during WRITE_DATA: "
		       << strerror(errno));
	m_state = WRITE_ERROR;
	return COM_OTHER_ERROR;
      }
      
      if (0 == nremain)	// can also be -1 if EAGAIN, will return COM_TRY_AGAIN at end
	m_state = READY;
      else {
	m_data_wip += nwritten;
	m_data_nremain -= nwritten;
      }
    }
    
    if (READY == m_state)
      return COM_OK;
    
    return COM_TRY_AGAIN;
  }


  NetSource::
  NetSource(int bufsize, int max_bufsize)
    : buffer(bufsize, max_bufsize)
  {
    Reset();
  }
  
  
  void NetSource::
  Reset()
  {
    m_state = READY;
    
    // the rest is overkill...
    m_length = -1;
    m_length_wip = 0;
    m_length_nremain = 0;
    m_data_wip = 0;
    m_data_nremain = 0;
  }
  
  
  com_status NetSource::
  Receive(int fd)
  {
    if (RESIZE_ERROR == m_state) {
      LOG_ERROR (logger, "wbcnet::NetSource::Receive(): RESIZE_ERROR state");
      return COM_OTHER_ERROR;
    }
    
    if (READ_ERROR == m_state) {
      LOG_ERROR (logger, "wbcnet::NetSource::Receive(): READ_ERROR state");
      return COM_OTHER_ERROR;
    }
    
    if (DONE == m_state)
      m_state = READY;
    
    if (READY == m_state) {
      m_length_wip = (char*) &m_length;
      m_length_nremain = sizeof(m_length);
      m_state = READ_LENGTH;
    }
    
    if (READ_LENGTH == m_state) {
      LOG_TRACE (logger,
		     "wbcnet::NetSource::Receive(): READ_LENGTH state\n"
		     << "  &m_length:        " << (unsigned long) &m_length << "\n"
		     << "  m_length_wip:     " << (unsigned long) m_length_wip << "\n"
		     << "  sizeof(m_length): " << sizeof(m_length) << "\n"
		     << "  m_length_nremain: " << m_length_nremain);
      int nread;
      int const
	nremain(com_readbuf(fd, m_length_wip, m_length_nremain, &nread));
      if ((-1 == nremain) && (EAGAIN != errno)) {
	LOG_ERROR (logger,
		       "wbcnet::NetSource::Receive(): com_readbuf() failed during READ_LENGTH: "
		       << strerror(errno));
	m_state = READ_ERROR;
	return COM_OTHER_ERROR;
      }
      
      if (0 == nremain)	// can also be -1 if EAGAIN, will return COM_TRY_AGAIN at end
	m_state = RESIZE;
      else {
	m_length_wip += nread;
	m_length_nremain -= nread;
      }
    }
    
    if (RESIZE == m_state) {
      LOG_TRACE (logger,
		     "wbcnet::NetSource::Receive(): RESIZE state\n"
		     << "  m_length:         " << m_length << "\n"
		     << "  m_length_nremain: " << m_length_nremain);
      if ( ! buffer.Resize(m_length)) {
	LOG_ERROR (logger,
		       "wbcnet::NetSource::Receive(): buffer.Resize(" << m_length << ") failed");
	m_state = RESIZE_ERROR;
	return COM_OTHER_ERROR;
      }
      m_data_wip = buffer.GetData();
      m_data_nremain = m_length;
      m_state = READ_DATA;
    }
    
    if (READ_DATA == m_state) {
      LOG_TRACE (logger,
		     "wbcnet::NetSource::Receive(): READ_DATA state\n"
		     << "  buffer.GetData(): " << (unsigned long) buffer.GetData() << "\n"
		     << "  m_data_wip:       " << (unsigned long) m_data_wip << "\n"
		     << "  buffer.GetSize(): " << buffer.GetSize() << "\n"
		     << "  m_length:         " << m_length << "\n"
		     << "  m_data_nremain:   " << m_data_nremain);
      int nread;
      int const
	nremain(com_readbuf(fd, m_data_wip, m_data_nremain, &nread));
      if ((-1 == nremain) && (EAGAIN != errno)) {
	LOG_ERROR (logger,
		       "wbcnet::NetSource::Receive(): com_readbuf() failed during READ_DATA: "
		       << strerror(errno));
	m_state = READ_ERROR;
	return COM_OTHER_ERROR;
      }
      
      if (0 == nremain)	// can also be -1 if EAGAIN, will return COM_TRY_AGAIN at end
	m_state = DONE;
      else {
	m_data_wip += nread;
	m_data_nremain -= nread;
      }
    }
    
    if (DONE == m_state)
      return COM_OK;
    
    return COM_TRY_AGAIN;
  }
  
}
