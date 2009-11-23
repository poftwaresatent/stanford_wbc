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

#ifndef WBC_SHUFFLE_VECTOR_HPP
#define WBC_SHUFFLE_VECTOR_HPP

#include <sys/types.h>
#include <vector>
#include <iosfwd>

class SAIVector;

namespace wbc {

  class ShuffleVector
  {
  public:
    explicit ShuffleVector(double empty_value);
  
    void SetShuffle(size_t from_index, size_t to_index);
  
    bool ShuffleDirect(double const * in, size_t in_len, double * out, size_t out_len) const;
    bool ShuffleInverse(double const * in, size_t in_len, double * out, size_t out_len) const;
  
    bool ShuffleDirect(SAIVector const & in, SAIVector & out) const;
    bool ShuffleInverse(SAIVector const & in, SAIVector & out) const;
  
    typedef std::vector<ssize_t> shuffle_t; // -1 means 'use empty value'
  
  protected:
    double m_empty_value;
    shuffle_t m_direct;
    shuffle_t m_inverse;
  };

}

#endif // WBC_SHUFFLE_VECTOR_HPP
