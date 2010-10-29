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

#include <wbcnet/endian.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace wbcnet;
using namespace std;

int main(int argc, char ** argv)
{
  bool ok(true);
  
  {
    uint8_t aa(0x12);
    endian_swap(aa);
    if (0x12 == aa)
      cout << "uint8_t passed\n";
    else {
      printf("uint8_t FAILED: expected 0x12 but got 0x%x\n", aa);
      ok = false;
    }
  }
  
  {
    uint16_t aa(0x1234);
    endian_swap(aa);
    if (0x3412 == aa)
      cout << "uint16_t passed\n";
    else {
      printf("uint16_t FAILED: expected 0x3412 but got 0x%x\n", aa);
      ok = false;
    }
  }
  
  {
    uint32_t aa(0x12345678);
    endian_swap(aa);
    if (0x78563412 == aa)
      cout << "uint32_t passed\n";
    else {
      printf("uint32_t FAILED: expected 0x78563412 but got 0x%lx\n", (long) aa);
      ok = false;
    }
  }
  
  {
    uint64_t aa(0x123456789abcdef0ULL);
    endian_swap(aa);
    if (0xf0debc9a78563412ULL == aa)
      cout << "uint64_t passed\n";
    else {
      printf("uint64_t FAILED: expected 0xf0debc9a78563412 but got 0x%llx\n",
	     (long long unsigned int) aa);
      ok = false;
    }
  }
  
  {
    char const arr[] = { 1, 2, 3,   4, 5, 6 };
    char const chk[] = { 3, 2, 1,   6, 5, 4 };
    char res[]       = { 7, 7, 7,   7, 7, 7 };
    endian_array_swap(2, 3, arr, res);
    cout << "arr:";
    for (int ii(0); ii < 6; ++ii)
      cout << " " << (int) arr[ii];
    cout << "\nchk:";
    for (int ii(0); ii < 6; ++ii)
      cout << " " << (int) chk[ii];
    cout << "\nres:";
    for (int ii(0); ii < 6; ++ii)
      cout << " " << (int) res[ii];    
    cout << "\n";
    for (int ii(0); ii < 6; ++ii)
      if (chk[ii] == res[ii])
	cout << "chk[" << ii << "] passed\n";
      else {
	printf("chk[%d] FAILED: expected %d but got %d\n", ii, (int) chk[ii], (int) res[ii]);
	ok = false;
      }
  }
  
  if ( ! ok) {
    cout << "\ndetected FAILURES\n\n";
    return EXIT_FAILURE;    
  }
  cout << "\nALL TESTS PASSED\n\n";
  return EXIT_SUCCESS;
}
