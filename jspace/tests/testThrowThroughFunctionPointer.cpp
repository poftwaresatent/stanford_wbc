// This file is placed in the public domain.

// g++ -Wall -o testThrowThroughFunctionPointer testThrowThroughFunctionPointer.cpp

#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>

static int this_function_throws(char const * msg) throw(std::runtime_error);
static int this_function_does_not_throw(char const * msg) throw(std::runtime_error);

typedef int (*func_t)(char const *);

int main(int argc, char ** argv)
{
  bool ok(true);
  int foo;
  
  try {
    foo = this_function_throws("direct call");
    printf("oops, should have thrown, but got %d\n", foo);
    ok = false;
  }
  catch (std::runtime_error const & ee) {
    printf("good, it threw: %s\n", ee.what());
  }
  
  try {
    foo = this_function_does_not_throw("direct call");
    printf("good, it did not throw: %d\n", foo);
  }
  catch (std::runtime_error const & ee) {
    printf("oops, it should not have thrown: %s\n", ee.what());
    ok = false;
  }
  
  struct func_s {
    func_t func;
    bool throws;
  };
  func_s check[] = {
    { this_function_throws, true },
    { this_function_does_not_throw, false }
  };
  
  for (int ii(0); ii < 2; ++ii) {
    try {
      foo = check[ii].func("through function pointer");
      if (check[ii].throws) {
	printf("oops, check[%d] should have thrown, but got %d\n", ii, foo);
	ok = false;
      }
      else {
	printf("good, check[%d] did not throw: %d\n", ii, foo);
      }
    }
    catch (std::runtime_error const & ee) {
      if (check[ii].throws) {
	printf("good, check[%d] threw: %s\n", ii, ee.what());
      }
      else {
	printf("oops, check[%d] should not have thrown: %s\n", ii, ee.what());
	ok = false;
      }
    }
  }
  
  if ( ! ok) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}


int this_function_throws(char const * msg) throw(std::runtime_error)
{
  throw std::runtime_error(std::string("this_function_throws(`") + msg + "`)");
  return 17;
}


int this_function_does_not_throw(char const * msg) throw(std::runtime_error)
{
  return 42;
}
