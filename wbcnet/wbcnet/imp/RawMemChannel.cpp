/*
 * Copyright (c) 2010 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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
   \file imp/RawMemChannel.cpp
*/

#include "RawMemChannel.hpp"
#include <wbcnet/data.hpp>
#include <string.h>


namespace wbcnet {
  
  
  RawMemChannel::
  RawMemChannel(void * data_pointer,
		int data_size)
    : m_data_pointer(data_pointer),
      m_data_size(data_size)
  {
  }
  
  
  com_status RawMemChannel::
  Send(BufferAPI const & buffer)
  {
    // Check if someone is just wrapping a BufferAPI around our data,
    // in which case we can assume they know what they're doing and
    // everything is supposedly already in order.
    char const * buffer_data(buffer.GetData());
    if (m_data_pointer == (void*) buffer_data) {
      return COM_OK;
    }
    
    // Otherwise be paranoid and check that the provided data matches
    // our data in size.
    if (buffer.GetSize() != m_data_size) {
      return COM_SIZE_MISMATCH;
    }
    
    memcpy(m_data_pointer, buffer_data, m_data_size);
    return COM_OK;
  }
  
  
  com_status RawMemChannel::
  Receive(BufferAPI & buffer)
  {
    // Check if someone is just wrapping a BufferAPI around our data,
    // in which case we can assume they know what they're doing and
    // everything is supposedly already in order.
    //
    // NOTE: We cannot hold on to the buffer data pointer because it
    // might possibly change when we call Resize() further down.
    if (m_data_pointer == (void*) buffer.GetData()) {
      return COM_OK;
    }
    
    // Otherwise be paranoid and try to resize to the correct size.
    if ( ! buffer.Resize(m_data_size)) {
      return COM_OTHER_ERROR;
    }
    
    memcpy(buffer.GetData(), m_data_pointer, m_data_size);
    return COM_OK;
  }
  
}
