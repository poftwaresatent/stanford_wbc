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

#include "pack.hpp"
#include "data.hpp"
#include "endian.hpp"
#include "proxy.hpp"
#include <wbcnet/log.hpp>
#include <string.h>

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));


namespace {
  
  bool please_swap(wbcnet::endian_mode_t endian_mode)
  {
    if (wbcnet::ENDIAN_DETECT == endian_mode)
      return wbcnet::detect_little_endian();
    return wbcnet::ENDIAN_ALWAYS_SWAP == endian_mode;
  }
  
}


namespace wbcnet {
  
  
  Packable::
  Packable(std::string const & _name, bool _packer_owned, bool _fixed_size)
    : name(_name),
      packer_owned(_packer_owned),
      fixed_size(_fixed_size)
  {
  }
  
  
  Packable::
  ~Packable()
  {
  }
  
  
  char * Packable::
  Pack(char * buffer, endian_mode_t endian_mode) const
  {
    int const nbytes(GetItemSize() * GetItemCount());
    if (please_swap(endian_mode))
      endian_array_swap(GetItemCount(), GetItemSize(), GetConstData(), buffer);
    else
      memcpy(buffer, GetConstData(), nbytes);

    LOG_TRACE (logger,
	       "wbcnet::Packable::Pack() " << name << ": "
	       << hexdump_buffer(buffer, nbytes));

    return buffer + nbytes;
  }
  
  
  char const * Packable::
  Unpack(char const * buffer, endian_mode_t endian_mode)
  {
    int const nbytes(GetItemSize() * GetItemCount());
    if (please_swap(endian_mode))
      endian_array_swap(GetItemCount(), GetItemSize(), buffer, GetData());
    else
      memcpy(GetData(), buffer, nbytes);

    LOG_TRACE (logger,
	       "wbcnet::Packable::Unpack() " << name << ": "
	       << hexdump_buffer(GetData(), nbytes));
    
    return buffer + nbytes;
  }
  
  
  Packer::
  Packer()
    : m_fixed_pack_size(0)
  {
  }
  
  
  Packer::
  ~Packer()
  {
    for (size_t ii(0); ii < m_field.size(); ++ii)
      if (m_field[ii]->packer_owned)
	delete m_field[ii];
  }
  
  
  void Packer::
  AddField(Packable * field)
  {
    m_field.push_back(field);
    if (field->fixed_size)
      m_fixed_pack_size += field->GetItemSize() * field->GetItemCount();
    else
      m_variable_field.push_back(field);
  }
  
  
  int Packer::
  GetPackSize() const
  {
    if (m_variable_field.empty())
      return m_fixed_pack_size;
    int sum(m_fixed_pack_size);
    for (field_table_t::const_iterator ivf(m_variable_field.begin());
	 ivf != m_variable_field.end(); ++ivf)
      sum += (*ivf)->GetItemSize() * (*ivf)->GetItemCount();
    return sum;
  }
  
  
  char * Packer::
  PackAll(char * buffer, endian_mode_t endian_mode) const
  {
    for (size_t ii(0); ii < m_field.size(); ++ii)
      buffer = m_field[ii]->Pack(buffer, endian_mode);
    return buffer;
  }
  
  
  char const * Packer::
  UnpackAll(char const * buffer, endian_mode_t endian_mode)
  {
    for (size_t ii(0); ii < m_field.size(); ++ii)
      buffer = m_field[ii]->Unpack(buffer, endian_mode);
    return buffer;
  }
  
  
  char * Packer::
  Skip(char * buffer) const
  {
    buffer += GetPackSize();
    return buffer;
  }
  
  
  VectorPack::
  VectorPack(std::string const & name, bool packer_owned, bool fixed_size,
	     VectorStorageAPI * _vec)
    : Packable(name, packer_owned, fixed_size),
      vec(_vec)
  {
  }
  
  
  char * VectorPack::
  GetData()
  {
    return (char*) vec->DataPointer();
  }
  
  
  int VectorPack::
  GetItemSize() const
  {
    return vec->ElementStorageSize();
  }
  
  
  int VectorPack::
  GetItemCount() const
  {
    return vec->NElements();
  }
  
  
  MatrixPack::
  MatrixPack(std::string const & name, bool packer_owned, bool fixed_size,
	     MatrixStorageAPI ** _mx)
    : Packable(name, packer_owned, fixed_size),
      mx(_mx)
  {
  }
  
  
  char * MatrixPack::
  GetData()
  {
    return (char*) (*mx)->DataPointer();
  }
  
  
  int MatrixPack::
  GetItemSize() const
  {
    return (*mx)->ElementStorageSize();
  }
  
  
  int MatrixPack::
  GetItemCount() const
  {
    return (*mx)->NRows() * (*mx)->NColumns();
  }


  TimestampPack::
  TimestampPack(std::string const & name, bool packer_owned, timestamp * _tstampPtr)
    : Packable(name, packer_owned, true),
      tstampPtr(_tstampPtr)
  {
  }
  
  
  char * TimestampPack::
  GetData()
  {
    return (char*) &tstampPtr->tv_sec;
  }
  
  
  int TimestampPack::
  GetItemSize() const
  {
    return sizeof(tstampPtr->tv_sec);
  }
  
  
  int TimestampPack::
  GetItemCount() const
  {
    return 2;
  }
  
}
