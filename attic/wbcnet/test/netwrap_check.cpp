#include <NetWrapper.h>

int main(int argc, char ** argv)
{
  NetWrapper::TCPSocket foo;
  foo.Connect("localhost", 8080, false);
  return 0;
}
