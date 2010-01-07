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

#include <wbcnet/misc/id.hpp>
#include <iostream>
#include <sstream>
#include <cstdlib>

using namespace wbcnet;
using namespace std;

int main(int argc, char ** argv)
{
  bool ok(true);
  ostringstream os;
  
  {
    IDLookup idl(true);
    try {
      cout << "foo: " << (int) idl.GetID("foo") << "\n";
      cout << "bar: " << (int) idl.GetID("bar") << "\n";
      idl.Add("baz", 42);
      cout << "baz: " << (int) idl.GetID("baz") << "\n";
      cout << "badabumm: " << (int) idl.GetID("badabumm") << "\n";
      if ("bar" != idl.GetName(idl.GetID("bar"))) {
	os << "auto_assign == true: idl.GetName() failed on \"bar\"\n";
	ok = false;
      }
      if ("baz" != idl.GetName(idl.GetID("baz"))) {
	os << "auto_assign == true: idl.GetName() failed on \"baz\"\n";
	ok = false;
      }
      unique_id_t const foo_id(idl.GetID("foo"));
      try {
	idl.Add("foo", foo_id);
      }
      catch (runtime_error const & ee) {
	os << "auto_assign == true: duplicate Add() check failed\n";
	ok = false;
      }
    }
    catch (runtime_error const & ee) {
      os << "auto_assign == true: unexpected runtime_error " << ee.what() << "\n";
      ok = false;
    }
  }
  
  {
    IDLookup idl(false);
    try {
      cout << "foo: " << (int) idl.GetID("foo") << "\n";
      os << "auto_assign == false: should have thrown on idl.GetID(\"foo\")\n";
      ok = false;
    }
    catch (IDLookup::name_error const & ee) {
      cout << " OK caught " << ee.what() << "\n";
    }
    catch (runtime_error const & ee) {
      os << "auto_assign == false: should have thrown name_error, but got runtime_error "
	 << ee.what() << "\n";
      ok = false;
    }
    idl.Add("foo", 17);
    try {
      cout << "foo: " << (int) idl.GetID("foo") << "\n";
      if ("foo" != idl.GetName(17)) {
	os << "auto_assign == false: idl.GetName() failed on \"foo\"\n";
	ok = false;
      }
    }
    catch (runtime_error const & ee) {
      os << "auto_assign == false: unexpected runtime_error " << ee.what() << "\n";
      ok = false;
    }
  }
  
  {
    try {
      idl::Add("blah", 99);
      os << "singleton: should have thrown not_initialized\n";
      ok = false;
    }
    catch (idl::not_initialized const & ee) {
      cout << " OK caught " << ee.what() << "\n";
    }
    catch (runtime_error const & ee) {
      os << "singleton: should have thrown not_initialized, but got runtime_error "
	 << ee.what() << "\n";
      ok = false;
    }
    idl::CreateAutoAssignSingleton();
    try {
      idl::Add("blah", 99);
      cout << "foo: " << (int) idl::GetID("foo") << "\n";
      cout << "bar: " << (int) idl::GetID("bar") << "\n";
      cout << "blah: " << (int) idl::GetID("blah") << "\n";
      if ("blah" != idl::GetName(99)) {
	os << "singleton: idl::GetName() failed on \"blah\"\n";
	ok = false;
      }
      if (99 != idl::GetID("blah")) {
	os << "singleton: idl::GetID() failed on 99\n";
	ok = false;
      }
    }
    catch (runtime_error const & ee) {
      os << "singleton: (1) unexpected runtime_error " << ee.what() << "\n";
      ok = false;
    }
    try {
      idl::CreateAutoAssignSingleton();
    }
    catch (runtime_error const & ee) {
      os << "singleton: (2) unexpected runtime_error " << ee.what() << "\n";
      ok = false;
    }
    try {
      idl::CreateSingleton();
      os << "singleton: should have thrown init_conflict\n";
      ok = false;
    }
    catch (idl::init_conflict const & ee) {
      cout << " OK caught " << ee.what() << "\n";      
    }
    catch (runtime_error const & ee) {
      os << "singleton: (3) unexpected runtime_error " << ee.what() << "\n";
      ok = false;
    }
    try {
      idl::DestroySingleton();
      idl::CreateSingleton();
    }
    catch (runtime_error const & ee) {
      os << "singleton: (4) unexpected runtime_error " << ee.what() << "\n";
      ok = false;
    }
    try {
      cout << "foo: " << (int) idl::GetID("foo") << "\n";
      os << "singleton: should have thrown on idl::GetID(\"foo\")\n";
      ok = false;
    }
    catch (IDLookup::name_error const & ee) {
      cout << " OK caught " << ee.what() << "\n";
    }
    catch (runtime_error const & ee) {
      os << "singleton: should have thrown name_error, but got runtime_error "
	 << ee.what() << "\n";
      ok = false;
    }
    idl::Add("foo", 17);
    try {
      cout << "foo: " << (int) idl::GetID("foo") << "\n";
      if ("foo" != idl::GetName(17)) {
	os << "singleton: idl::GetName() failed on \"foo\"\n";
	ok = false;
      }
    }
    catch (runtime_error const & ee) {
      os << "singleton: (5) unexpected runtime_error " << ee.what() << "\n";
      ok = false;
    }
  }
  
  if ( ! ok) {
    cout << os.str() << "\ndetected FAILURES\n\n";
    return EXIT_FAILURE;    
  }
  cout << "\nALL TESTS PASSED\n\n";
  return EXIT_SUCCESS;
}
