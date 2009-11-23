#define DISABLE_LOGGING
#include <wbcnet/log.hpp>
#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("blah"));

int main(int argc, char ** argv)
{
  std::cerr << "AAA: There should be no output between lines `AAA' and `BBB'\n";
  LOG_TRACE(logger, "a TRACE message " << 42);
  LOG_DEBUG(logger, "a DEBUG message " << 42);
  LOG_INFO(logger,  "an INFO message " << 42);
  LOG_WARN(logger,  "a WARN message " << 42);
  LOG_ERROR(logger, "an ERROR message " << 42);
  LOG_FATAL(logger, "a FATAL message " << 42);
  std::cerr << "BBB: There should be no output between lines `AAA' and `BBB'\n";
}
