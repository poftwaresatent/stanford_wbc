#undef DISABLE_LOGGING
#define HAVE_LOG4CXX
#include <wbcnet/log.hpp>
#include <iostream>

static wbcnet::logger_t logger(wbcnet::get_logger("blah"));

int main(int argc, char ** argv)
{
  wbcnet::configure_logging();
  wbcnet::get_root_logger()->setLevel(log4cxx::Level::getInfo());
  std::cerr << "There should be INFO...FATAL messages, but no TRACE or DEBUG messages:\n";
  LOG_TRACE(logger, "a TRACE message " << 42);
  LOG_DEBUG(logger, "a DEBUG message " << 42);
  LOG_INFO(logger,  "an INFO message " << 42);
  LOG_WARN(logger,  "a WARN message " << 42);
  LOG_ERROR(logger, "an ERROR message " << 42);
  LOG_FATAL(logger, "a FATAL message " << 42);
}
