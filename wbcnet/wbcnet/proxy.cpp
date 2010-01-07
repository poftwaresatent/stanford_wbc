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

#include "proxy.hpp"
#include <wbcnet/log.hpp>
#ifndef WIN32
#include <sys/time.h>
#else
#include"win32/win32_compat.hpp"
#endif

#include <time.h>
////#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace wbcnet {
  
  
  char const * proxy_status_str(proxy_status ps)
  {
    static char const * str[] = {
      "PROXY_OK",
      "PROXY_WRONG_MSG_ID",
      "PROXY_RESIZE_ERROR",
      "PROXY_SIZE_MISMATCH",
      "PROXY_DIMENSION_MISMATCH",
      "PROXY_STORAGE_MISMATCH"      
    };
    if ((0 > ps) || (PROXY_STORAGE_MISMATCH < ps))
      return "(invalid proxy_status)";
    return str[ps];
  }
  
  
  proxy_status proxy_unpack_msg_id(BufferAPI const & buffer, unique_id_t & msg_id,
				   endian_mode_t endian_mode)
  {
    ValuePack<unique_id_t, fixed_count> vp("msg_id", false, &msg_id, 1);
    if (vp.GetItemSize() > buffer.GetSize())
      return PROXY_SIZE_MISMATCH;
    // Well... as long as msg_id is an 8-bit value, endian swapping is
    // a no-op, and we could just say ENDIAN_NEVER_SWAP here. But if
    // the msg_id type ever needs endian awareness, this will
    // introduce tricky bugs.
    vp.Unpack(buffer.GetData(), endian_mode);
    return PROXY_OK;
  }
  
  
  Proxy::
  Proxy(unique_id_t _id)
    : id(_id),
      m_tmpid(_id)
  {
    m_idpack.AddField(new ValuePack<unique_id_t, fixed_count>("Proxy::msg_id", true, &m_tmpid, 1));
  }
  
  
  int Proxy::
  GetPackSize()
  {
    return m_idpack.GetPackSize() + m_header.GetPackSize() + m_payload.GetPackSize();
  }
  
  
  proxy_status Proxy::
  Pack(BufferAPI & buffer, endian_mode_t endian_mode)
  {
    if ( ! buffer.Resize(GetPackSize()))
      return PROXY_RESIZE_ERROR;
    
    // m_idpack refers to m_tmpid, which must be reloaded before each
    // packing, because it is used in CheckID()
    m_tmpid = id;
    char * data(buffer.GetData());
    data = m_idpack.PackAll(data, endian_mode);
    data = m_header.PackAll(data, endian_mode);
    m_payload.PackAll(data, endian_mode);
    
    return PROXY_OK;
  }
  
  
  proxy_status Proxy::
  CheckID(BufferAPI const & buffer, endian_mode_t endian_mode)
  {
    if (m_idpack.GetPackSize() > buffer.GetSize())
      return PROXY_SIZE_MISMATCH;
    
    // m_idpack unpacks into m_tmpid, which is then available for
    // comparison
    m_idpack.UnpackAll(buffer.GetData(), endian_mode);
    if (id != m_tmpid)
      return PROXY_WRONG_MSG_ID;
    
    return PROXY_OK;
  }
  
  
  proxy_status Proxy::
  CheckHeader() const
  {
    return PROXY_OK;
  }
  
  
  proxy_status Proxy::
  UnpackHeader(BufferAPI const & buffer, endian_mode_t endian_mode)
  {
    if (m_idpack.GetPackSize() + m_header.GetPackSize() > buffer.GetSize())
      return PROXY_SIZE_MISMATCH;
    char const * data(buffer.GetData());
    data = m_idpack.SkipConst(data);
    m_header.UnpackAll(data, endian_mode);
    return CheckHeader();
  }
  
  
  proxy_status Proxy::
  UnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode)
  {
    return DoUnpackPayload(buffer, endian_mode);
  }
  
  
  proxy_status Proxy::
  DoUnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode)
  {
    if (m_payload.IsEmpty())
      return PROXY_OK;
    if (GetPackSize() > buffer.GetSize()) {
      LOG_ERROR (logger,
		     "size mismatch in wbcnet::Proxy::DoUnpackPayload()\n"
		     << "  unique_id:        " << (int) id << "\n"
		     << "  GetPackSize():    " << GetPackSize() << "\n"
		     << "    id:             " << m_idpack.GetPackSize() << "\n"
		     << "    header:         " << m_header.GetPackSize() << "\n"
		     << "    payload:        " << m_payload.GetPackSize() << "\n"
		     << "  buffer.GetSize(): " << buffer.GetSize());
      return PROXY_SIZE_MISMATCH;
    }
    char const * data(buffer.GetData());
    data = m_idpack.SkipConst(data);
    data = m_header.SkipConst(data);
    m_payload.UnpackAll(data, endian_mode);
    return PROXY_OK;
  }
  
  
  timestamp::
  timestamp()
    : tv_sec(0),
      tv_usec(0)
  {
  }
  
  
  timestamp::
  timestamp(uint32_t _tv_sec, uint32_t _tv_usec)
    : tv_sec(_tv_sec),
      tv_usec(_tv_usec)
  {
  }
  
  
  timestamp::
  timestamp(timeval const & tv)
    : tv_sec(tv.tv_sec),
      tv_usec(tv.tv_usec)
  {
  }
  
  
  timestamp::
  operator timeval () const
  {
    timeval res;
    res.tv_sec = tv_sec;
    res.tv_usec = tv_usec;
    return res;
  }
  
  
  timestamp & timestamp::
  operator = (timeval const & tv)
  {
    tv_sec = tv.tv_sec;
    tv_usec = tv.tv_usec;
    return *this;
  }
    
  
  bool timestamp::
  operator == (timestamp const & rhs) const
  {
    return (tv_sec == rhs.tv_sec ) && (tv_usec == rhs.tv_usec);
  }
  
  
  bool timestamp::
  gettimeofday(struct timezone *tz)
  {
    timeval tt;
    if ( 0 != ::gettimeofday(&tt, tz))
      return false;
    *this = tt;
    return true;
  }
  
}
