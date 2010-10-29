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

/** \file testPack.cpp Unit tests for Packer and related classes. */

#include <wbcnet/pack.hpp>
#include <wbcnet/data.hpp>
#include <iostream>
#include <sstream>
#include <math.h>
#include <stdlib.h>

using namespace wbcnet;
using namespace std;

static const endian_mode_t endian_mode(ENDIAN_DETECT);


bool test_fixed()
{
  static int const checkfoo(42);
  static double const checkbar(17.17);
  Buffer buf(0, -1);
  
  {
    int foo(checkfoo);
    double bar(checkbar);
    Packer packer;
    packer.AddField(new ValuePack<int, fixed_count>("foo", true, &foo, 1));
    packer.AddField(new ValuePack<double, fixed_count>("bar", true, &bar, 1));
    if ( ! buf.Resize(packer.GetPackSize())) {
      cout << "test_fixed(): buf.Resize(" << packer.GetPackSize() << ") failed\n";
      return false;
    }
    cout << "test_fixed(): packing\n";
    packer.PackAll(buf.GetData(), endian_mode);
  }
  
  bool ok(true);
  {
    int foo(17 - checkfoo);
    double bar(4.2 * checkbar);
    Packer packer;
    packer.AddField(new ValuePack<int, fixed_count>("foo", true, &foo, 1));
    packer.AddField(new ValuePack<double, fixed_count>("bar", true, &bar, 1));
    if (buf.GetSize() != packer.GetPackSize()) {
      cout << "test_fixed(): buf.GetSize() == " << buf.GetSize() << " != packer.GetPackSize() == "
	   << packer.GetPackSize() << "\n";
      return false;
    }
    cout << "test_fixed(): unpacking\n";
    packer.UnpackAll(buf.GetData(), endian_mode);
    cout << "test_fixed(): checking\n";
    if (foo != checkfoo) {
      ok = false;
      cout << "test_fixed(): foo == " << foo << " != checkfoo == " << checkfoo << "\n";
    }
    if (bar != checkbar) {
      ok = false;
      cout << "test_fixed(): bar == " << bar << " != checkbar == " << checkbar << "\n";
    }
  }
  
  return ok;
}


bool test_variable()
{
  static int const max_count(17);
  static int const count[] = { 17, 4 };
  static double const seed[] = { 3.141, -888.8 };
  Buffer buf(0, -1);
  
  {
    double foo[max_count];
    ValuePack<double, variable_count> bar("foo", false, foo, max_count);
    Packer packer;
    packer.AddField(new ValuePack<int, fixed_count>("bar.count", true, &bar.count, 1));
    packer.AddField(&bar);
    if (sizeof(int) + max_count * sizeof(double) != packer.GetPackSize()) {
      cout << "test_variable(): sizeof(int) + max_count * sizeof(double) == "
	   << sizeof(int) + max_count * sizeof(double)
	   << " != packer.GetPackSize() == " << packer.GetPackSize() << "\n";
      return false;
    }
    if ( ! buf.Resize( 2 * packer.GetPackSize())) {
      cout << "test_variable(): buf.Resize(" << 2 * packer.GetPackSize() << ") failed\n";
      return false;
    }
    cout << "test_variable: buffer resized to " << buf.GetSize() << " bytes\n"
	 << "test_variable(): packing\n";
    char * data(buf.GetData());
    for (int ii(0); ii < 2; ++ii) {
      bar.count = count[ii];
      for (int jj(0); jj < count[ii]; ++jj) {
	foo[jj] = jj * seed[ii];
	cout << " " << foo[jj];
      }
      cout << "\n";
      data = packer.PackAll(data, endian_mode);
    }
  }
  
  bool ok(true);
  {
    double foo[max_count];
    ValuePack<double, variable_count> bar("foo", false, foo, max_count);
    Packer packer;
    packer.AddField(new ValuePack<int, fixed_count>("bar.count", true, &bar.count, 1));
    packer.AddField(&bar);
    if (buf.GetSize() != 2 * packer.GetPackSize()) {
      cout << "test_variable(): buf.GetSize() == " << buf.GetSize()
	   << " != 2 * packer.GetPackSize() == "
	   << 2 * packer.GetPackSize() << "\n";
      return false;
    }
    char const * data(buf.GetData());
    for (int ii(0); ii < 2; ++ii) {
      cout << "test_variable(): unpacking\n";
      data = packer.UnpackAll(data, endian_mode);
      cout << "test_variable(): checking\n";
      if (bar.count != count[ii]) {
	ok = false;
	cout << "test_variable(): bar.count == " << bar.count << " != count[" << ii << "] == "
	     << count[ii] << "\n";
      }
      else {
	for (int jj(0); jj < count[ii]; ++jj) {
	  cout << " " << foo[jj];
	  double const checkfoo(jj * seed[ii]);
	  if (foo[jj] != checkfoo) {
	    ok = false;
	    cout << "!=" << checkfoo;
	  }
	}
	cout << "\n";
      }
    }
  }
  
  return ok;
}


int main(int argc, char ** argv)
{
  ostringstream os;
  bool ok(true);
  
  if (test_fixed())
    os << "test_fixed() passed\n";
  else {
    ok = false;
    os << "test_fixed() FAILED\n";
  }
  
  if (test_variable())
    os << "test_variable() passed\n";
  else {
    ok = false;
    os << "test_variable() FAILED\n";
  }
  
  cout << "==================================================\n"
       << os.str();
  if ( ! ok) {
    cout << "\ndetected FAILURES\n\n";
    return EXIT_FAILURE;    
  }
  cout << "\nALL TESTS PASSED\n\n";
  return EXIT_SUCCESS;
}
