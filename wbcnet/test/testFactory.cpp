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

#include <wbcnet/Factory.hpp>
#include <gtest/gtest.h>

using namespace wbcnet;
using namespace std;

class Foo {
public:
  virtual ~Foo() {}
  virtual int Blah() = 0;
};

class Foo1: public Foo {
public:
  virtual int Blah() { return 1; }
};

class Foo2: public Foo {
public:
  virtual int Blah() { return 2; }
};


TEST (factory, typestuff)
{
  typedef Factory<Foo1, Foo> foo1_factory_t;
  typedef Factory<Foo2, Foo> foo2_factory_t;
  typedef FactoryAPI<Foo> foo_factory_api_t;
  {
    foo1_factory_t ff1;
    foo2_factory_t ff2;
    foo_factory_api_t * ff;
    ff = &ff1;
    ff = &ff2;
  }
}


TEST (factory, registry)
{
  FactoryRegistry<Foo> ff;
  try {
    ff.Add("1", new Factory<Foo1, Foo>());
    ff.Add("2", new Factory<Foo2, Foo>());
  }
  catch (...) {
    ASSERT_TRUE (false) << "adding factories should not have thrown";
  }
  ASSERT_TRUE (ff.Have("1"));
  ASSERT_TRUE (ff.Have("2"));
  ASSERT_FALSE (ff.Have("3"));
}


TEST (factory, creation)
{
  FactoryRegistry<Foo> ff;
  try {
    Foo * f1;
    f1 = ff.Create("1");
    ASSERT_TRUE (false) << "invoking non-existing factory should have thrown";
  }
  catch (...) {
  }
  ff.Add("1", new Factory<Foo1, Foo>());
  ff.Add("2", new Factory<Foo2, Foo>());
  Foo * f1(ff.Create("1"));
  Foo * f2(ff.Create("2"));
  ASSERT_EQ (f1->Blah(), 1) << "f1 is of wrong type";
  ASSERT_EQ (f2->Blah(), 2) << "f2 is of wrong type";
  delete f1;
  delete f2;
}


class Bar: public Foo {
public:
  explicit Bar(int _bar): bar(_bar) {}
  virtual int Blah() { return bar; }
  int bar;
};

class BarFactory: public FactoryAPI<Foo> {
public:
  explicit BarFactory(int _bar): bar(_bar) {}
  virtual Bar * Create() { return new Bar(bar); }
  int bar;
};

TEST (factory, custom_ctor)
{
  FactoryRegistry<Foo> ff;
  ff.Add("17", new BarFactory(17));
  ff.Add("42", new BarFactory(42));
  Foo * b17(ff.Create("17"));
  Foo * b42(ff.Create("42"));
  ASSERT_EQ (b17->Blah(), 17) << "b17 has wrong data";
  ASSERT_EQ (b42->Blah(), 42) << "b42 has wrong data";
  delete b17;
  delete b42;
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
