#include <log4cxx/logmanager.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/consoleappender.h>

using namespace log4cxx;

int main(int argc, char ** argv)
{
  LogManager::getLoggerRepository()->setConfigured(true);
  LoggerPtr root(Logger::getRootLogger());
  static const LogString TTCC_CONVERSION_PATTERN(LOG4CXX_STR("%d %-5p %c - %m%n"));
  LayoutPtr layout(new PatternLayout(TTCC_CONVERSION_PATTERN));
  AppenderPtr appender(new ConsoleAppender(layout));
  root->addAppender(appender);
  root->setLevel(log4cxx::Level::getWarn());
  return 0;
}
