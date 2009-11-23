ACLOCAL_AMFLAGS= -I m4

CPPFLAGS=      -I@abs_top_srcdir@ @CPPFLAGS@
CXXFLAGS=      $(CPPFLAGS) @CXXFLAGS@
LDFLAGS=       @LDFLAGS@

##################################################
# conditionals

if HAVE_MQUEUE
  MQUEUE_LIBADD=  -lrt
  MQUEUE_SRCS= wbcnet/MQWrap.cpp
  MQUEUE_HDRS= wbcnet/MQWrap.hpp
endif

if HAVE_LOG4CXX
  LOG4CXX_LIBADD= -llog4cxx
endif

if HAVE_NETWRAP
  NETWRAP_LIBADD= -lnetwrapper
  NETWRAP_SRCS= wbcnet/NetWrapperWrap.cpp
  NETWRAP_HDRS= wbcnet/NetWrapperWrap.hpp
endif

##################################################

lib_LTLIBRARIES=      libwbcnet.la
libwbcnet_la_SOURCES= wbcnet/msg/StringList.cpp \
                      wbcnet/msg/Matrix.cpp \
                      wbcnet/msg/TaskSpec.cpp \
                      wbcnet/msg/UserCommand.cpp \
                      wbcnet/msg/Status.cpp \
                      wbcnet/msg/RobotState.cpp \
                      wbcnet/msg/TaskMatrix.cpp \
                      wbcnet/msg/ServoCommand.cpp \
                      wbcnet/SPQueue.cpp \
                      wbcnet/log.cpp \
                      wbcnet/Muldex.cpp \
                      wbcnet/TaskAtomizer.cpp \
                      wbcnet/proxy.cpp \
                      wbcnet/pack.cpp \
                      wbcnet/DelayHistogram.cpp \
                      wbcnet/StreamBufMgr.cpp \
                      wbcnet/strutil.cpp \
                      wbcnet/data.cpp \
                      wbcnet/com.cpp \
                      wbcnet/endian.cpp \
                      wbcnet/id.cpp \
                      $(MQUEUE_SRCS) \
                      $(NETWRAP_SRCS)
libwbcnet_la_LDFLAGS= $(LDFLAGS)
libwbcnet_la_LIBADD=  $(MQUEUE_LIBADD) \
                      $(LOG4CXX_LIBADD) \
                      $(NETWRAP_LIBADD)

include_HEADERS= wbcnet/AutoSocket.hpp \
                 wbcnet/DelayHistogram.hpp \
                 wbcnet/MQWrap.hpp \
                 wbcnet/Muldex.hpp \
                 wbcnet/SPQueue.hpp \
                 wbcnet/SockWrap.hpp \
                 wbcnet/StreamBufMgr.hpp \
                 wbcnet/TaskAtomizer.hpp \
                 wbcnet/com.hpp \
                 wbcnet/data.hpp \
                 wbcnet/endian.hpp \
                 wbcnet/endian_mode.hpp \
                 wbcnet/fake_log.hpp \
                 wbcnet/id.hpp \
                 wbcnet/log.hpp \
                 wbcnet/net.hpp \
                 wbcnet/pack.hpp \
                 wbcnet/pdebug.hpp \
                 wbcnet/proxy.hpp \
                 wbcnet/strutil.hpp \
                 wbcnet/udp_util.hpp \
                 wbcnet/msg/Matrix.hpp \
                 wbcnet/msg/RobotState.hpp \
                 wbcnet/msg/ServoCommand.hpp \
                 wbcnet/msg/Status.hpp \
                 wbcnet/msg/StringList.hpp \
                 wbcnet/msg/TaskMatrix.hpp \
                 wbcnet/msg/TaskSpec.hpp \
                 wbcnet/msg/UserCommand.hpp \
                 $(MQUEUE_HDRS) \
                 $(NETWRAP_HDRS)
includedir=      @includedir@/wbcnet
