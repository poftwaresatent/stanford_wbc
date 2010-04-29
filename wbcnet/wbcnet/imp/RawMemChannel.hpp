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
   \file imp/RawMemChannel.hpp Raw memory single-"message"
   "communication" for single-process applications.
*/

#ifndef WBCNET_RAW_MEM_CHANNEL_HPP
#define WBCNET_RAW_MEM_CHANNEL_HPP

#include <wbcnet/com.hpp>


namespace wbcnet {
  
  
  class Buffer;
  
  
  /**
     Simple raw memory based "communication". Sending is simulated by
     overwriting a memory zone provided by the client, receiving by
     reading from that zone. If you read twice in a row, you get the
     same "message" each time. If you write twice in a row, you
     overwrite what you've written earlier.
  */
  class RawMemChannel
    : public Channel
  {
  public:
    RawMemChannel(/** Pointer to user-supplied and user-managed memory. */
		  void * data_pointer,
		  /** Number of bytes in the user-supplied memory. */
		  int data_size);
    
    virtual com_status Send(BufferAPI const & buffer);
    virtual com_status Receive(BufferAPI & buffer);
    
  protected:
    void * m_data_pointer;
    int const m_data_size;
  };
  
  
  /**
     Convenience template for wrapping data structures that are
     already consecutive chunks of memory. The memory must still be
     managed by the user.
  */
  template<typename custom_ds_t>
  class CustomMemChannel
    : public RawMemChannel
  {
  public:
    typedef custom_ds_t data_type;
    
    explicit CustomMemChannel(data_type * data)
      : RawMemChannel(data, sizeof(data_type)) {}
    
    inline data_type * GetData()
    { return reinterpret_cast<data_type*>(m_data_pointer); }
    
    inline data_type const * GetData() const
    { return reinterpret_cast<data_type const *>(m_data_pointer); }
  };
  
}

#endif // WBCNET_RAW_MEM_CHANNEL_HPP
