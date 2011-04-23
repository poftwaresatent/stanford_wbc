// This file is placed in the public domain.

// g++ -Wall -g -O0 -o test_std_string test_std_string.cpp

#include <string>
#include <iostream>

int main(int argc, char ** argv)
{
  static char const * foo_cstr("fooXXX");
  std::string foo("something else");
  foo.resize(3);
  foo.replace(0, 3, foo_cstr);
  if (foo != "foo") {
    std::cout << "foo.replace(0, 3, foo_cstr) failed: got \"" << foo << "\" instead of \"foo\"\n";
  }
  foo.resize(3);
  foo.replace(0, 3, foo_cstr, 3);
  if (foo != "foo") {
    std::cout << "foo.replace(0, 3, foo_cstr, 3) failed: got \"" << foo << "\" instead of \"foo\"\n";
  }
}
