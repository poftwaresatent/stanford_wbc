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

#include "data.hpp"
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef WIN32
# define snprintf sprintf_s
#endif


namespace wbcnet {
  
  
  /**
     \todo Check for malloc() errors.
  */
  Buffer::
  Buffer(int size, int max_size)
    : m_data(0),
      m_alloc_size(size),
      m_apparent_size(size),
      m_max_size(max_size)
  {
    if (m_alloc_size > 0)
      m_data = reinterpret_cast<char *>(malloc(m_alloc_size));
    if ((max_size >= 0) && (max_size < m_alloc_size))
      max_size = m_alloc_size;
  }
  
  
  /**
     \todo Check for malloc() errors.
  */
  Buffer::
  Buffer(Buffer const & orig)
    : m_data(0),
      m_alloc_size(orig.m_apparent_size),
      m_apparent_size(orig.m_apparent_size),
      m_max_size(orig.m_max_size)
  {
    if (m_alloc_size > 0) {
      m_data = reinterpret_cast<char *>(malloc(m_alloc_size));
      memcpy(m_data, orig.m_data, m_alloc_size);
    }
  }
  
  
  /**
     \todo Check for malloc() errors.
  */
  Buffer::
  Buffer(BufferAPI const & orig)
    : m_data(0),
      m_alloc_size(orig.GetSize()),
      m_apparent_size(orig.GetSize()),
      m_max_size(orig.GetSize())
  {
    if (m_alloc_size > 0) {
      m_data = reinterpret_cast<char *>(malloc(m_alloc_size));
      memcpy(m_data, orig.GetData(), m_alloc_size);
    }
  }
  
  
  Buffer::
  ~Buffer()
  {
    free(m_data);
  }
  
  
  char * Buffer::
  GetData() const
  {
    return m_data;
  }
  
  
  int Buffer::
  GetSize() const
  {
    return m_apparent_size;
  }
  
  
  bool Buffer::
  Resize(int size)
  {
    if (m_apparent_size == size)
      return true;
    if (0 > size)
      return false;
    if (m_alloc_size >= size) {
      m_apparent_size = size;
      return true;
    }
    if ((m_max_size >= 0) && (size > m_max_size))
      return false;
    if (0 == m_data) {
      m_data = reinterpret_cast<char *>(malloc(size));
      if (0 == m_data)
	return false;
    }
    else {
      char * tmp(reinterpret_cast<char *>(realloc(m_data, size)));
      if (0 == tmp)
	return false;
      m_data = tmp;
    }
    m_alloc_size = size;
    m_apparent_size = size;
    return true;
  }
  
  
  bool Buffer::
  Set(BufferAPI const & rhs)
  {
    if ( ! Resize(rhs.GetSize()))
      return false;
    if (m_apparent_size > 0)
      memcpy(m_data, rhs.GetData(), m_apparent_size);
    return true;
  }
  
  
  bool VectorStorageAPI::
  operator == (VectorStorageAPI const & rhs) const
  {
    if (NElements() != rhs.NElements())
      return false;
    if (ElementStorageSize() != rhs.ElementStorageSize())
      return false;
    if (0 == memcmp(DataPointer(), rhs.DataPointer(), NElements() * ElementStorageSize()))
      return true;
    return false;
  }
  
  
  void VectorStorageAPI::
  SetZero()
  {
    if (NElements() > 0)
      memset(DataPointer(), 0, NElements() * ElementStorageSize());
  }
  
  
  bool VectorStorageAPI::
  Copy(VectorStorageAPI const & rhs)
  {
    if (&rhs == this)
      return true;
    if (ElementStorageSize() != rhs.ElementStorageSize())
      return false;
    if ((NElements() != rhs.NElements()) && ( ! SetNElements(rhs.NElements())))
      return false;
    memcpy(DataPointer(), rhs.DataPointer(), NElements() * ElementStorageSize());
    return true;
  }
  
  
  bool VectorStorageAPI::
  Splice(int dst_begin_idx, VectorStorageAPI const & src, int src_begin_idx, int src_end_idx)
  {
    int const elem_size(ElementStorageSize());
    if (elem_size != src.ElementStorageSize())
      return false;
    
    // adjust dst_begin_index
    if (0 > dst_begin_idx)
      dst_begin_idx = NElements() - dst_begin_idx;
    if (0 > dst_begin_idx)
      dst_begin_idx = 0;
    
    // adjust src_begin_idx and src_end_idx
    int const src_nelem(src.NElements());
    if (0 > src_begin_idx)
      src_begin_idx = src_nelem - src_begin_idx;
    if (0 > src_begin_idx)
      src_begin_idx = 0;
    if (0 > src_end_idx)
      src_end_idx = src_nelem - src_end_idx;
    if (src_nelem <= src_end_idx)
      src_end_idx = src_nelem;
    
    // compute size of splice (before and after), treat negative size
    // as empty range
    int src_len(src_end_idx - src_begin_idx);
    if (0 > src_len)
      src_len = 0;
    
    // resize destination (this can create non-initialized values in
    // case of a dst_begin_index that is higher than the size of the
    // destination before this call to Splice... however, the caller
    // is expected to know what they are doing).
    if ( ! SetNElements(dst_begin_idx + src_len))
      return false;
    
    // do the splice, unless it is an empty range. NOTE: use memmove
    // because src might be *this
    if (0 != src_len)
      memmove(DataPointer()     + dst_begin_idx * elem_size,
	      src.DataPointer() + src_begin_idx * elem_size,
	                          src_len       * elem_size);
    return true;
  }
  
  
  bool MatrixStorageAPI::
  operator == (MatrixStorageAPI const & rhs) const
  {
    if (NRows() != rhs.NRows())
      return false;
    if (NColumns() != rhs.NColumns())
      return false;
    if (ElementStorageSize() != rhs.ElementStorageSize())
      return false;
    // XXXX maybe one day also check both are row-major or column-major
    if (0 == memcmp(DataPointer(), rhs.DataPointer(), NRows() * NColumns() * ElementStorageSize()))
      return true;
    return false;
  }
  
  
  void MatrixStorageAPI::
  SetZero()
  {
    if ((NRows() > 0) && (NColumns() > 0))
      memset(DataPointer(), 0, NRows() * NColumns() * ElementStorageSize());
  }
  
  
  bool MatrixStorageAPI::
  Copy(MatrixStorageAPI const & rhs)
  {
    if (&rhs == this)
      return true;
    if (ElementStorageSize() != rhs.ElementStorageSize())
      return false;
    if (((NRows() != rhs.NRows()) || (NColumns() != rhs.NColumns()))
	&& ( ! SetSize(rhs.NRows(), rhs.NColumns())))
      return false;
    memcpy(DataPointer(), rhs.DataPointer(), NRows() * NColumns() * ElementStorageSize());
    return true;
  }
  
  
  bool MatrixStorageAPI::
  Copy(VectorStorageAPI const & rhs)
  {
    if (ElementStorageSize() != rhs.ElementStorageSize())
      return false;
    if (((NRows() != rhs.NElements()) || (NColumns() != 1))
	&& ( ! SetSize(rhs.NElements(), 1)))
      return false;
    memcpy(DataPointer(), rhs.DataPointer(), NRows() * ElementStorageSize());
    return true;
  }
  
  
  std::string hexdump_buffer(char const * buffer, int nbytes)
  {
    std::ostringstream result;
    char hex[4];
    hex[3] = '\0';
    for (int ii(0); ii < nbytes; ++ii) {
      snprintf(hex, 3, "%02x", buffer[ii]);
      if (ii > 0)
	result << " ";
      result << hex;
    }
    return result.str();
  }

}
