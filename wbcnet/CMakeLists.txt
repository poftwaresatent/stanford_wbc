CMAKE_MINIMUM_REQUIRED (VERSION 2.4)
IF (COMMAND CMAKE_POLICY)
  CMAKE_POLICY (SET CMP0003 NEW)
  CMAKE_POLICY (SET CMP0002 NEW)
ENDIF (COMMAND CMAKE_POLICY)

PROJECT (WBCNET)
SET (CMAKE_VERBOSE_MAKEFILE ON)

message (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
message ("configuring WBCNET")

LIST (APPEND WBCNET_SRCS
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
      wbcnet/msg/StringList.cpp
      wbcnet/msg/RobotState.cpp
      wbcnet/msg/ServoCommand.cpp
      wbcnet/msg/Status.cpp
      wbcnet/msg/Matrix.cpp
      wbcnet/msg/TaskMatrix.cpp
      wbcnet/msg/TaskSpec.cpp
      wbcnet/msg/Service.cpp
      wbcnet/msg/UserCommand.cpp)

LIST (APPEND CMAKE_REQUIRED_INCLUDES /usr/local/include)
INCLUDE_DIRECTORIES (/usr/local/include)
LINK_DIRECTORIES (/usr/local/lib)

INCLUDE (CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG (-pipe WBCNET_CXX_FLAG_pipe)
IF (WBCNET_CXX_FLAG_pipe)
  ADD_DEFINITIONS (-pipe)
ENDIF (WBCNET_CXX_FLAG_pipe)

CHECK_CXX_COMPILER_FLAG (-Wall WBCNET_CXX_FLAG_Wall)
IF (WBCNET_CXX_FLAG_Wall)
  ADD_DEFINITIONS (-Wall)
ENDIF (WBCNET_CXX_FLAG_Wall)

IF (NOT BUILD_SHARED_LIBS)
   SET (BUILD_SHARED_LIBS True)
ENDIF (NOT BUILD_SHARED_LIBS)

IF (NOT CMAKE_BUILD_TYPE)
   SET (CMAKE_BUILD_TYPE Debug)
ENDIF (NOT CMAKE_BUILD_TYPE)

IF (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
   ADD_DEFINITIONS (-DWBCNET_DEBUG)
   CHECK_CXX_COMPILER_FLAG (-O0 WBCNET_CXX_FLAG_O0)
   IF (WBCNET_CXX_FLAG_O0)
     ADD_DEFINITIONS (-O0)
   ENDIF (WBCNET_CXX_FLAG_O0)
ENDIF (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

LIST (APPEND WBCNET_LIBS dl)

INCLUDE (CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX (mqueue.h WBCNET_HAVE_MQUEUE)
IF (${WBCNET_HAVE_MQUEUE})
	ADD_DEFINITIONS (-DWBCNET_HAVE_MQUEUE)
	LIST (APPEND WBCNET_SRCS wbcnet/imp/MQWrap.cpp)
	LIST (APPEND WBCNET_LIBS rt)
ENDIF (${WBCNET_HAVE_MQUEUE})

# try to get LOG4CXX_DIR from CMake or environment
# 1. CMake variable LOG4CXX_DIR
# 2. environment variable LOG4CXX_DIR
# 3. environment variable ROS_BINDEPS_PATH
# 4. give up
if (NOT LOG4CXX_DIR)
  set (LOG4CXX_DIR $ENV{LOG4CXX_DIR})
endif (NOT LOG4CXX_DIR)
if (NOT LOG4CXX_DIR)
  set (LOG4CXX_DIR $ENV{ROS_BINDEPS_PATH})
endif (NOT LOG4CXX_DIR)
if (LOG4CXX_DIR)
  message ("Using LOG4CXX_DIR = ${LOG4CXX_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
  list (APPEND TRYCOMPILE_INCLUDE_DIRECTORIES ${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
  include_directories (${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
  list (APPEND TRYCOMPILE_LINK_DIRECTORIES ${LOG4CXX_DIR}/lib ${LOG4CXX_DIR})
  link_directories (${LOG4CXX_DIR}/lib ${LOG4CXX_DIR})
endif (LOG4CXX_DIR)

check_include_file_cxx (log4cxx/logger.h WBCNET_HAVE_LOG4CXX)
if (WBCNET_HAVE_LOG4CXX)
  message ("found log4cxx headers, will check it's library version")
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
    message ("log4cxx version check succeeded")
  else (LOG4CXX_VERSION_CHECK)
    message ("LOG4CXX_VERSION_CHECK_LOG:\n${LOG4CXX_VERSION_CHECK_LOG}")
    message (FATAL_ERROR "The log4cxx version check failed (see try_compile log above). Please install log4cxx-0.10.0 (e.g. from the 3rdparty/ directory of stanford-wbc) and tell CMake where to find it by setting the LOG4CXX_DIR variable (either in the environment or by passing -DLOG4CXX:path=/foo/bar to cmake)")
  endif (LOG4CXX_VERSION_CHECK)
  add_definitions (-DHAVE_LOG4CXX)
  list (APPEND WBCNET_LIBS log4cxx)
else (WBCNET_HAVE_LOG4CXX)
  message ("WARNING did not find log4cxx, will use simplistic logging")
endif (WBCNET_HAVE_LOG4CXX)

# try to get NETWRAP_DIR from CMake or environment
# 1. CMake variable NETWRAP_DIR
# 2. environment variable NETWRAP_DIR
if (NOT NETWRAP_DIR)
  set (NETWRAP_DIR $ENV{NETWRAP_DIR})
  if (NETWRAP_DIR)
    message ("using NETWRAP_DIR from environment: ${NETWRAP_DIR}")
  endif (NETWRAP_DIR)
else (NOT NETWRAP_DIR)
  message ("using NETWRAP_DIR from CMake: ${NETWRAP_DIR}")
endif (NOT NETWRAP_DIR)
if (NETWRAP_DIR)
  message ("DBG NETWRAP_DIR is ${NETWRAP_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${NETWRAP_DIR}/include)
  include_directories (${NETWRAP_DIR}/include)
  link_directories (${NETWRAP_DIR}/lib)
  message ("DBG CMAKE_REQUIRED_INCLUDES: ${CMAKE_REQUIRED_INCLUDES}")
  check_include_file_cxx (NetWrapper.h WBCNET_HAVE_NETWRAP)
  message ("DBG WBCNET_HAVE_NETWRAP: ${WBCNET_HAVE_NETWRAP}")
  if (WBCNET_HAVE_NETWRAP)
    message ("found NetWrapper.h")
    message ("WARNING: skipping the try_compile() step because CMake seems broken...")
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
#      message ("checked compilation and linking of Forcedimension LLC's NetWrapper library")
#    else (NETWRAP_CHECK)
#      message ("NETWRAP_CHECK_LOG:\n${NETWRAP_CHECK_LOG}")
#      message (FATAL_ERROR "The netwrapper check failed (see try_compile log above).")
#    endif (NETWRAP_CHECK)
    add_definitions (-DHAVE_NETWRAP)
    list (APPEND WBCNET_LIBS netwrapper)
    list (APPEND WBCNET_SRCS wbcnet/imp/NetWrapperWrap.cpp)
  else (WBCNET_HAVE_NETWRAP)
    message (FATAL_ERROR "NetWrapper.h not found even though NETWRAP_DIR was set to ${NETWRAP_DIR}")
  endif (WBCNET_HAVE_NETWRAP)
endif (NETWRAP_DIR)

include_directories (${WBCNET_SOURCE_DIR})

add_library (wbcnet SHARED ${WBCNET_SRCS})
target_link_libraries (wbcnet ${WBCNET_LIBS})
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
target_link_libraries (testSockWrapMuldex wbcnet ${WBCNET_LIBS})

add_executable (testEndian test/testEndian.cpp)
target_link_libraries (testEndian wbcnet ${WBCNET_LIBS})

add_executable (testPack test/testPack.cpp)
target_link_libraries (testPack wbcnet wbcnet ${WBCNET_LIBS})

add_executable (testProxy test/testProxy.cpp)
target_link_libraries (testProxy wbcnet ${WBCNET_LIBS})

add_executable (testTaskAtomizer test/testTaskAtomizer.cpp)
target_link_libraries (testTaskAtomizer wbcnet ${WBCNET_LIBS})

add_executable (testStreamBufMgr test/testStreamBufMgr.cpp)
target_link_libraries (testStreamBufMgr wbcnet ${WBCNET_LIBS})

add_executable (testID test/testID.cpp)
target_link_libraries (testID wbcnet ${WBCNET_LIBS})

add_executable (testMuldex test/testMuldex.cpp)
target_link_libraries (testMuldex wbcnet ${WBCNET_LIBS})

add_executable (testDelayHistogram test/testDelayHistogram.cpp)
target_link_libraries (testDelayHistogram wbcnet ${WBCNET_LIBS})

add_executable (testLogDisabled test/testLogDisabled.cpp)
target_link_libraries (testLogDisabled wbcnet ${WBCNET_LIBS})

add_executable (testLogWithoutLog4cxx test/testLogWithoutLog4cxx.cpp)
target_link_libraries (testLogWithoutLog4cxx wbcnet ${WBCNET_LIBS})

add_library (TestModule MODULE test/TestModule.cpp)
add_executable (testTestModule test/testTestModule.cpp)
if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
  target_link_libraries (testTestModule wbcnet ${WBCNET_LIBS})
else (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
  target_link_libraries (testTestModule dl wbcnet ${WBCNET_LIBS})
endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)

if (${WBCNET_HAVE_MQUEUE})
  add_executable (testMQWrap test/testMQWrap.cpp)
  target_link_libraries (testMQWrap wbcnet ${WBCNET_LIBS})
  add_executable (testMQSpeed test/testMQSpeed.cpp)
  target_link_libraries (testMQSpeed wbcnet ${WBCNET_LIBS})
endif (${WBCNET_HAVE_MQUEUE})

if (WBCNET_HAVE_LOG4CXX)
  add_executable (testLogWithLog4cxx test/testLogWithLog4cxx.cpp)
  target_link_libraries (testLogWithLog4cxx wbcnet ${WBCNET_LIBS})
endif (WBCNET_HAVE_LOG4CXX)

if (WBCNET_HAVE_NETWRAP)
  add_executable (testNetWrapper test/testNetWrapper.cpp)
  target_link_libraries (testNetWrapper wbcnet ${WBCNET_LIBS})
endif (WBCNET_HAVE_NETWRAP)

### try to find Google testing framework, and enable some more tests if it was found

message (STATUS "GTEST_DIR is \"${GTEST_DIR}\"")
if (GTEST_DIR)
  message (STATUS "using GTEST_DIR")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
  include_directories (${GTEST_DIR}/include ${GTEST_DIR})
  link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
endif (GTEST_DIR)

check_include_file_cxx (gtest/gtest.h WBCNET_HAVE_GTEST)

if (${WBCNET_HAVE_GTEST})
  message (STATUS "Found Google testing framework.")
  #XXXX to do: resurrect test_model_servo
  ###  add_executable (test_model_servo test/test_model_servo.cpp test/TestDirectory.cpp)
  ###  target_link_libraries (test_model_servo wbcnet ${WBCNET_LIBS} gtest)
  add_executable (testFactory test/testFactory.cpp)
  target_link_libraries (testFactory wbcnet ${WBCNET_LIBS} gtest)
else (${WBCNET_HAVE_GTEST})
  message (STATUS "WARNING Google testing framework not found! If you do have it, use -DGTEST_DIR:path=/where/ever or set the GTEST_DIR environment variable")
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
