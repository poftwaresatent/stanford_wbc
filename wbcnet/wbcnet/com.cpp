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

#include "com.hpp"
#include "data.hpp"
#include <wbcnet/log.hpp>
#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
//#include <unistd.h>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  char const * com_status_str(com_status cs)
  {
    static char const * str[] = {
      "COM_OK",
      "COM_NOT_OPEN",
      "COM_NOT_CONNECTED",
      "COM_NOT_BOUND",
      "COM_TRY_AGAIN",
      "COM_SIZE_MISMATCH",
      "COM_INTERRUPTED",
      "COM_TIMEOUT",
      "COM_OTHER_ERROR"
    };
    if ((0 > cs) || (COM_OTHER_ERROR < cs))
      return "(invalid com_status)";
    return str[cs];
  }
  
  
#ifndef WIN32

  int com_writebuf(int fd, char const * data, int n_bytes,
		   int * n_written)
  {
    LOG_TRACE (logger, "wbcnet::com_writebuf(): n_bytes " << n_bytes);
    
    int nw;
    if (n_written)
      *n_written = 0;
    while ((nw = write(fd, data, n_bytes)) != -1 && 0 != nw) {
      LOG_TRACE (logger, "  wrote " << hexdump_buffer(data, nw));
      n_bytes -= nw;
      data += nw;
      if (n_written)
	*n_written += nw;
    }
    if (-1 == nw)
      return -1;
    return n_bytes; // zero if everything was written, because of n_bytes -= nw above
  }
  
  
  int com_readbuf(int fd, char * data, int n_bytes,
		  int * n_read)
  {
    LOG_TRACE (logger, "wbcnet::com_readbuf(): n_bytes " << n_bytes);

    int nr;
    if (n_read)
      *n_read = 0;
    while ((nr = read(fd, data, n_bytes)) != -1 && 0 != nr) {
      LOG_TRACE (logger, "  read " << hexdump_buffer(data, nr));
      n_bytes -= nr;
      data += nr;
      if (n_read)
	*n_read += nr;
    }
    if (-1 == nr)
      return -1;
    return n_bytes; // zero if everything was read, because of n_bytes -= nr above
  }
  
#endif // WIN32

  
  ProxyChannel::
  ProxyChannel(Sink * sink,
	       bool own_sink,
	       Source * source,
	       bool own_source)
    : m_sink(sink),
      m_own_sink(own_sink),
      m_source(source),
      m_own_source(own_source)
  {
  }
  
  
  ProxyChannel::
  ~ProxyChannel()
  {
    if (m_own_sink)
      delete m_sink;
    if (m_own_source)
      delete m_source;
  }
  
  
  com_status ProxyChannel::
  Send(BufferAPI const & buffer)
  {
    return m_sink->Send(buffer);
  }
  
  
  com_status ProxyChannel::
  Receive(BufferAPI & buffer)
  {
    return m_source->Receive(buffer);
  }
  
}
