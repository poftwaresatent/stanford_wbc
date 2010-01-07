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
   \file proxy.hpp Transparent serializing and deserializing support.
   
   Generic and (half-) specific data structures that support
   transparent serializing and deserializing. One of the major reasons
   for writing the wbcnet library was to produce the Proxy class
   declared in this file.
*/

#ifndef WBCNET_PROXY_HPP
#define WBCNET_PROXY_HPP

#include <wbcnet/misc/id.hpp>
#include <wbcnet/data.hpp>
#include <wbcnet/pack.hpp>

struct timeval;
struct timezone;

namespace wbcnet {
  
  
  /**
     Return value definition of several proxy functions. You can use
     proxy_status_str() to get a (statically allocated) string
     describing a particular status.
  */
  typedef enum {
    PROXY_OK,
    PROXY_WRONG_MSG_ID,
    PROXY_RESIZE_ERROR,
    PROXY_SIZE_MISMATCH,
    PROXY_DIMENSION_MISMATCH,
    PROXY_STORAGE_MISMATCH
  } proxy_status;
  
  /**
     \return A statically allocated string to the symbolic name of a
     proxy_status. Useful for creating human-reabable error messages.
  */
  char const * proxy_status_str(proxy_status ps);
  
  
  /**
     Proxies are (supposed to be) uniquely identified by message
     ID. This function allows you to peek inside some data and find
     out it's message ID, which is useful e.g. inside dispatchers such
     as MdxDispatcher::DemuxOne().
  */
  proxy_status proxy_unpack_msg_id(BufferAPI const & buffer, unique_id_t & msg_id,
				   endian_mode_t endian_mode);
  
  
  /**
     Base class of serializable data. Each "message" or synchronizable
     data structure is subdivided into three parts:
     - a message ID, supposedly unique for this structure
     - a fixed-size header, subdivided into fields
     - an optional variable-sized payload (it can also be fixed sized if you want)
     
     Subclasses of Proxy should, inside their constructors, call
     Packer::AddFixedField() and Packer::AddVariableField() as
     appropriate on the Packer::m_header and Packer::m_payload
     fields. They also have to implement UnpackPayload() and maybe
     provide different Pack() and UnpackHeader(). Optionally,
     subclasses can add supplementary checks on the header data by
     overriding CheckHeader(), which gets called at the end of
     UnpackHeader().
     
     Note that the code does not enfore the absence of variable header
     fields... if you want that, you're on your own in figuring out
     how to avoid memory corruption. (This means you must not do
     m_header.AddField() with a non-fixed size data structure).
  */
  class Proxy
  {
  public:
    explicit Proxy(/** supposedly unique message ID */
		   unique_id_t id);
    virtual ~Proxy() {}
    
    /**
       \return The combined sizes of all fixed and variable fields,
       which includes the message ID and the header.
    */
    int GetPackSize();
    
    /**
       Serialize everything (the ID, header, and payload) into a
       buffer. Well, actually it just delegates to Packer::PackAll(),
       which delegates to all its Packable::Pack(), which do the real
       actual work.
    */
    virtual proxy_status Pack(BufferAPI & buffer, endian_mode_t endian_mode);
    
    /**
       De-serialize the header fields from a buffer. This is delegated
       to Packer::UnpackAll(), and then the result of CheckHeader() is
       returned. By default, CheckHeader() returns PROXY_OK, but
       subclasses can override this in case they want to implement
       checks on NDOF or so.
       
       \note When you use wbcnet utilities such as MdxDispatcher, you
       don't have to worry about checking the ID or making sure you
       get called in the correct order.
    */
    virtual proxy_status UnpackHeader(BufferAPI const & buffer, endian_mode_t endian_mode);
    
    /**
       Hook for de-serializing payload fields from a buffer. This
       default implementation simply delegates to
       DoUnpackPayload(). Most subclasses should be fine with this,
       but maybe you want to do some things before (such as checking
       the message ID) or afterwards.
    */
    virtual proxy_status UnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode);
    
    /**
       Verify that the message ID stored in a buffer corresponds to
       ours (Proxy::id).
    */
    proxy_status CheckID(BufferAPI const & buffer, endian_mode_t endian_mode);
    
    /**
       Called at the end of UnpackHeader(). By default, this method
       returns PROXY_OK, which usually -- some calls later -- ends up
       telling UnpackPayload() to proceed with overwriting subclass
       data with whatever payload is in the buffer.
    */
    virtual proxy_status CheckHeader() const;
    
    unique_id_t const id;
    
  protected:
    /**
       Do the actual work of unpacking the payload fields from a
       buffer.  Checks that the payload is not empty, then delegates
       to Packer::UnpackAll().
    */
    proxy_status DoUnpackPayload(BufferAPI const & buffer, endian_mode_t endian_mode);
    
    Packer m_header;
    Packer m_payload;
    
  private:
    unique_id_t m_tmpid;
    Packer m_idpack;
  };
  
  
  /**
     Our very own timeval look-alike. The trouble with timeval is that
     it's hard to be sure all operating systems and platforms use the
     same format, so here we make sure that we send two unsigned
     32-bit values. Otherwise it should be fully compatible with your
     system's struct timeval.
  */
  struct timestamp {
    timestamp();
    timestamp(uint32_t tv_sec, uint32_t tv_usec);
    explicit timestamp(struct timeval const & tv);    
    
    operator timeval () const;    
    timestamp & operator = (timeval const & tv);    
    bool operator == (timestamp const & rhs) const;    
    bool gettimeofday(struct timezone *tz);
    
    // Do not insert fields between tv_sec tv_usec because
    // TimestampPack assumes them to be contiguous. Do not make them
    // differently-sized either.
    uint32_t tv_sec;  // 188 years starting in 1970: should last till 2158
    uint32_t tv_usec;
  };
  
}

#endif // WBCNET_PROXY_HPP
