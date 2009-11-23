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

/** \file endian.hpp Utilities for handling byte-order conversions. */

#ifndef WBCNET_ENDIAN_HPP
#define WBCNET_ENDIAN_HPP

#ifdef WIN32
# include "win32_stdint.hpp"
#else
# include <stdint.h>
#endif

namespace wbcnet {
  
  
  inline bool detect_big_endian() {
    static uint16_t const foo(0x1234);
    return ((*reinterpret_cast<uint8_t const *>(&foo)) == 0x12);
  }
  
  
  inline bool detect_little_endian() {
    static uint16_t const foo(0x1234);
    return ((*reinterpret_cast<uint8_t const *>(&foo)) == 0x34);
  }
  
  
  template<int nbytes>
  struct endian_swap_imp {
  };
  
  
  template<typename TT>
  void endian_swap(TT & value) {
    endian_swap_imp<sizeof(TT)>::convert(reinterpret_cast<char*>(&value));
  }
  
  
  void endian_array_swap(int n_items, int item_size,
			 char const * native,
			 char * swapped);
  
  
  template<>
  struct endian_swap_imp<1> {
    static inline void convert(char * value) {}
  };
  
  
  template<>
  struct endian_swap_imp<2> {
    static inline void convert(char * value) {
      char const tmp(value[0]);
      value[0] = value[1];
      value[1] = tmp; }
  };
  
  
  template<>
  struct endian_swap_imp<4> {
    static inline void convert(char * value) {
      char tmp(value[0]);
      value[0] = value[3];
      value[3] = tmp;
      tmp = value[1];
      value[1] = value[2];
      value[2] = tmp; }
  };
  
  
  template<>
  struct endian_swap_imp<8> {
    static inline void convert(char * value) {
      char tmp(value[0]);
      value[0] = value[7];
      value[7] = tmp;
      tmp = value[1];
      value[1] = value[6];
      value[6] = tmp;
      tmp = value[2];
      value[2] = value[5];
      value[5] = tmp;
      tmp = value[3];
      value[3] = value[4];
      value[4] = tmp; }
  };

}

#endif // WBCNET_ENDIAN_HPP
