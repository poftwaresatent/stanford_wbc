#include <wbc/util/dtor_check.hpp>
using namespace wbc;

struct Foo {
  virtual ~Foo() { oops.check(this); }
  dtor_check oops;
};

struct Bar {
  Bar(Foo * foo): my_foo(foo) {}
  ~Bar() { delete my_foo; }
  Foo * my_foo;
};

struct Toto {
  Toto(Foo * foo): my_foo(foo) {}
  ~Toto() { delete my_foo; }
  Foo * my_foo;
};

int main() {
  Foo * foo(new Foo());
  Bar * bar(new Bar(foo));
  Toto * toto(new Toto(foo));
  delete toto;
  delete bar;
}
