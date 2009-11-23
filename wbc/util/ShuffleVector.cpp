/*
 * Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include "ShuffleVector.hpp"
#include <saimatrix/SAIVector.h>
#include <wbcnet/log.hpp>

static wbcnet::logger_t logger(wbcnet::get_logger("wbc"));


namespace wbc {

  ShuffleVector::
  ShuffleVector(double empty_value)
    : m_empty_value(empty_value)
  {
  }


  void ShuffleVector::
  SetShuffle(size_t from_index, size_t to_index)
  {
    LOG_TRACE (logger, "ShuffleVector::SetShuffle(" << from_index << ", " << to_index << ")");
  
    if (m_direct.size() <= to_index) {
      size_t const oldsize(m_direct.size());
      m_direct.resize(to_index + 1);
      LOG_TRACE (logger, "  resizing m_direct to " << m_direct.size() << " entries");
      for (size_t ii(oldsize); ii < to_index; ++ii)
	m_direct[ii] = -1;
    }
    m_direct[to_index] = from_index;
    LOG_TRACE (logger, "  m_direct[" << to_index << "] = " << from_index);

    if (m_inverse.size() <= from_index) {
      size_t const oldsize(m_inverse.size());
      m_inverse.resize(from_index + 1);
      LOG_TRACE (logger, "  resizing m_inverse to " << m_inverse.size() << " entries");
      for (size_t ii(oldsize); ii < from_index; ++ii)
	m_inverse[ii] = -1;
    }
    m_inverse[from_index] = to_index;
    LOG_TRACE (logger, "  m_inverse[" << from_index << "] = " << to_index);
  }


  static bool do_shuffle(ShuffleVector::shuffle_t const & shuffle,
			 double empty_value,
			 double const * in, size_t in_len, double * out, size_t out_len)
  {
    bool ok(true);
    for (size_t to_index(0); to_index < shuffle.size(); ++to_index) {
      ssize_t const from_index(shuffle[to_index]);
      LOG_TRACE (logger, "ShuffleVector do_shuffle(): from " << from_index << " to " << to_index);
    
      if (out_len <= to_index) {
	LOG_TRACE (logger, "  oops: " << out_len << " = out_len <= to_index = " << to_index);
	return false;
      }
    
      if (0 > from_index)
	out[to_index] = empty_value;
      else {
	if (in_len <= from_index) {
	  LOG_TRACE (logger, "  oops: " << in_len << " = in_len <= from_index = " << from_index);
	  ok = false;
	}
	else
	  out[to_index] = in[from_index];
      }
    }
    return ok;
  }


  bool ShuffleVector::
  ShuffleDirect(double const * in, size_t in_len, double * out, size_t out_len) const
  {
    return do_shuffle(m_direct, m_empty_value, in, in_len, out, out_len);
  }


  bool ShuffleVector::
  ShuffleInverse(double const * in, size_t in_len, double * out, size_t out_len) const
  {
    return do_shuffle(m_inverse, m_empty_value, in, in_len, out, out_len);
  }


  bool ShuffleVector::
  ShuffleDirect(SAIVector const & in, SAIVector & out) const
  {
    return do_shuffle(m_direct, m_empty_value, in.dataPtr(), in.size(),
		      out.dataPtr(), out.size());
  }


  bool ShuffleVector::
  ShuffleInverse(SAIVector const & in, SAIVector & out) const
  {
    return do_shuffle(m_inverse, m_empty_value, in.dataPtr(), in.size(),
		      out.dataPtr(), out.size());
  }

}
