cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (WBCNET)

list (APPEND WBCNET_SRCS
      wbcnet/log.cpp
      wbcnet/pack.cpp
      wbcnet/com.cpp
      wbcnet/data.cpp
      wbcnet/proxy.cpp
      wbcnet/net.cpp
      wbcnet/endian.cpp
      wbcnet/misc/id.cpp
      wbcnet/misc/StreamBufMgr.cpp
      wbcnet/imp/SPQueue.cpp
      wbcnet/misc/TaskAtomizer.cpp
      wbcnet/imp/SockWrap.cpp
      wbcnet/Muldex.cpp
      wbcnet/strutil.cpp
      wbcnet/misc/DelayHistogram.cpp
      wbcnet/misc/AutoSocket.cpp
      wbcnet/misc/udp_util.cpp
      wbcnet/NetConfig.cpp
      wbcnet/misc/message_id.cpp
      wbcnet/DLModule.cpp
      wbcnet/imp/MQNetConfig.cpp
      wbcnet/imp/TCPNetConfig.cpp
      wbcnet/imp/SPQNetConfig.cpp
      wbcnet/imp/NetWrapNetConfig.cpp
      wbcnet/imp/RawMemChannel.cpp
      wbcnet/msg/StringList.cpp
      wbcnet/msg/RobotState.cpp
      wbcnet/msg/ServoCommand.cpp
      wbcnet/msg/Status.cpp
      wbcnet/msg/Matrix.cpp
      wbcnet/msg/TaskMatrix.cpp
      wbcnet/msg/TaskSpec.cpp
      wbcnet/msg/Service.cpp
      wbcnet/msg/UserCommand.cpp)

list (APPEND WBCNET_LIBS dl)

include (CheckIncludeFileCXX)
check_include_file_cxx (mqueue.h WBCNET_HAVE_MQUEUE)
if (${WBCNET_HAVE_MQUEUE})
	add_definitions (-DWBCNET_HAVE_MQUEUE)
	list (APPEND WBCNET_SRCS wbcnet/imp/MQWrap.cpp)
	list (APPEND WBCNET_LIBS rt)
endif (${WBCNET_HAVE_MQUEUE})

if (LOG4CXX_DIR STREQUAL "DISABLED")
  message ("[WBCNET] logging has been explicitly disabled")
  add_definitions (-DDISABLE_LOGGING)
else (LOG4CXX_DIR STREQUAL "DISABLED")
  check_include_file_cxx (log4cxx/logger.h WBCNET_HAVE_LOG4CXX)
  if (WBCNET_HAVE_LOG4CXX)
    message ("[WBCNET] found log4cxx headers, will check it's library version")
    try_compile (LOG4CXX_VERSION_CHECK
      ${WBCNET_BINARY_DIR}/log4cxx_version_check
      ${WBCNET_SOURCE_DIR}/test/log4cxx_version_check.cpp
      CMAKE_FLAGS
        -DINCLUDE_DIRECTORIES:path=${TRYCOMPILE_INCLUDE_DIRECTORIES}
	-DLINK_DIRECTORIES:path=${TRYCOMPILE_LINK_DIRECTORIES}
	-DLINK_LIBRARIES:string=log4cxx
      OUTPUT_VARIABLE LOG4CXX_VERSION_CHECK_LOG
      )
    if (LOG4CXX_VERSION_CHECK)
      message ("[WBCNET] log4cxx version check succeeded")
    else (LOG4CXX_VERSION_CHECK)
      message ("[WBCNET] LOG4CXX_VERSION_CHECK_LOG:\n${LOG4CXX_VERSION_CHECK_LOG}")
      message (FATAL_ERROR "[WBCNET] The log4cxx version check failed (see try_compile log above). Please install log4cxx-0.10.0 (e.g. from the 3rdparty/ directory of stanford-wbc) and tell CMake where to find it by setting the LOG4CXX_DIR variable (either in the environment or by passing -DLOG4CXX_DIR:path=/foo/bar to cmake). If that fails to do the trick, try disabling it by passing -DLOG4CXX_DIR=DISABLED.")
    endif (LOG4CXX_VERSION_CHECK)
    add_definitions (-DHAVE_LOG4CXX)
    list (APPEND WBCNET_LIBS log4cxx)
  else (WBCNET_HAVE_LOG4CXX)
    # We're raising a fatal error above, so this warning will never be
    # reached... but let's keep it here in case at some point we
    # decide to treat missing log4cxx as a warning instead of an
    # error.
    message ("[WBCNET] WARNING did not find log4cxx, will use simplistic logging")
  endif (WBCNET_HAVE_LOG4CXX)
endif (LOG4CXX_DIR STREQUAL "DISABLED")

check_include_file_cxx (NetWrapper.h WBCNET_HAVE_NETWRAP)
message ("[WBCNET] DBG WBCNET_HAVE_NETWRAP: ${WBCNET_HAVE_NETWRAP}")
if (WBCNET_HAVE_NETWRAP)
  message ("[WBCNET] found NetWrapper.h")
  message ("[WBCNET] WARNING: skipping the try_compile() step because CMake seems broken...")
  #    try_compile (NETWRAP_CHECK
  #      ${WBCNET_BINARY_DIR}/netwrap_check
  #      ${WBCNET_SOURCE_DIR}/test/netwrap_check.cpp
  #      CMAKE_FLAGS
  #        -DINCLUDE_DIRECTORIES:path=${NETWRAP_DIR}/include
  #	-DLINK_DIRECTORIES:path=${NETWRAP_DIR}/lib
  #	-DLINK_LIBRARIES:string=netwrapper
  #      OUTPUT_VARIABLE NETWRAP_CHECK_LOG
  #      )
  #    if (NETWRAP_CHECK)
  #      message ("[WBCNET] checked compilation and linking of Forcedimension LLC's NetWrapper library")
  #    else (NETWRAP_CHECK)
  #      message ("[WBCNET] NETWRAP_CHECK_LOG:\n${NETWRAP_CHECK_LOG}")
  #      message (FATAL_ERROR "[WBCNET] The netwrapper check failed (see try_compile log above).")
  #    endif (NETWRAP_CHECK)
  add_definitions (-DHAVE_NETWRAP)
  list (APPEND WBCNET_LIBS netwrapper)
  list (APPEND WBCNET_SRCS wbcnet/imp/NetWrapperWrap.cpp)
endif (WBCNET_HAVE_NETWRAP)

include_directories (${WBCNET_SOURCE_DIR})

add_library (wbcnet SHARED ${WBCNET_SRCS})
target_link_libraries (wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})
if (WBCNET_HAVE_NETWRAP)
  # This little thinggie is supposed to make it easier for users of
  # wbcnet to find the libraries that wbcnet is linked against,
  # e.g. log4cxx and netwrapper.
  set_target_properties (wbcnet
    PROPERTIES
    INSTALL_RPATH ${NETWRAP_DIR}/lib
    BUILD_WITH_INSTALL_RPATH True)
endif (WBCNET_HAVE_NETWRAP)

add_executable (testSockWrapMuldex test/testSockWrapMuldex.cpp)
target_link_libraries (testSockWrapMuldex wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testEndian test/testEndian.cpp)
target_link_libraries (testEndian wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testPack test/testPack.cpp)
target_link_libraries (testPack wbcnet wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testProxy test/testProxy.cpp)
target_link_libraries (testProxy wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testTaskAtomizer test/testTaskAtomizer.cpp)
target_link_libraries (testTaskAtomizer wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testStreamBufMgr test/testStreamBufMgr.cpp)
target_link_libraries (testStreamBufMgr wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testID test/testID.cpp)
target_link_libraries (testID wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testMuldex test/testMuldex.cpp)
target_link_libraries (testMuldex wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testDelayHistogram test/testDelayHistogram.cpp)
target_link_libraries (testDelayHistogram wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testLogDisabled test/testLogDisabled.cpp)
target_link_libraries (testLogDisabled wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_executable (testLogWithoutLog4cxx test/testLogWithoutLog4cxx.cpp)
target_link_libraries (testLogWithoutLog4cxx wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})

add_library (TestModule MODULE test/TestModule.cpp)
target_link_libraries (TestModule ${MAYBE_GCOV})
add_executable (testTestModule test/testTestModule.cpp)
if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
  target_link_libraries (testTestModule wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})
else (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
  target_link_libraries (testTestModule dl wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})
endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)

if (${WBCNET_HAVE_MQUEUE})
  add_executable (testMQWrap test/testMQWrap.cpp)
  target_link_libraries (testMQWrap wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})
  add_executable (testMQSpeed test/testMQSpeed.cpp)
  target_link_libraries (testMQSpeed wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})
endif (${WBCNET_HAVE_MQUEUE})

if (WBCNET_HAVE_LOG4CXX)
  add_executable (testLogWithLog4cxx test/testLogWithLog4cxx.cpp)
  target_link_libraries (testLogWithLog4cxx wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})
endif (WBCNET_HAVE_LOG4CXX)

if (WBCNET_HAVE_NETWRAP)
  add_executable (testNetWrapper test/testNetWrapper.cpp)
  target_link_libraries (testNetWrapper wbcnet ${WBCNET_LIBS} ${MAYBE_GCOV})
endif (WBCNET_HAVE_NETWRAP)

check_include_file_cxx (gtest/gtest.h WBCNET_HAVE_GTEST)
if (${WBCNET_HAVE_GTEST})
  message ("[WBCNET] Found Google testing framework.")
  add_executable (testFactory test/testFactory.cpp)
  target_link_libraries (testFactory wbcnet ${WBCNET_LIBS} gtest ${MAYBE_GCOV})
else (${WBCNET_HAVE_GTEST})
  message ("[WBCNET] WARNING Google testing framework not found! If you do have it, use -DGTEST_DIR:path=/where/ever or set the GTEST_DIR environment variable")
endif (${WBCNET_HAVE_GTEST})

### installation-related commands

FILE (GLOB headers "wbcnet/*.hpp")
INSTALL (FILES ${headers} DESTINATION include/wbcnet)

FILE (GLOB msg_headers "wbcnet/msg/*.hpp")
INSTALL (FILES ${msg_headers} DESTINATION include/wbcnet/msg)

FILE (GLOB imp_headers "wbcnet/imp/*.hpp")
INSTALL (FILES ${imp_headers} DESTINATION include/wbcnet/imp)

FILE (GLOB misc_headers "wbcnet/misc/*.hpp")
INSTALL (FILES ${misc_headers} DESTINATION include/wbcnet/misc)

FILE (GLOB win32_headers "wbcnet/win32/*.hpp")
INSTALL (FILES ${win32_headers} DESTINATION include/wbcnet/win32)

### use e.g. "cmake -DCMAKE_INSTALL_PREFIX=`pwd`/stage" to specify an
### installation directory other than /usr/local

INSTALL (TARGETS wbcnet
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
