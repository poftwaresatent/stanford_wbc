cmake_minimum_required (VERSION 2.4)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0002 NEW)
endif (COMMAND CMAKE_POLICY)

project (WBCRUN)
set (CMAKE_VERBOSE_MAKEFILE ON)

message (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
message ("configuring WBCRUN")


### the core sources and dependencies

list (APPEND WBCRUN_SRCS
      util.cpp
      message_id.cpp
      service.cpp
      directory.cpp
      DLModule.cpp
      Process.cpp
      UserProcess.cpp)

### set up some system-dependent things...

message ("System: ${CMAKE_SYSTEM_NAME}")

if (WIN32)
  message ("Detected Microsoft Windows")
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    message ("Detected Mac OS X")
    add_definitions (-DOSX)
  endif (APPLE)
  
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    message ("Detected Linux")
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    message ("Detected OpenBSD")
    add_definitions (-DOPENBSD)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)


### some compiler flags are not understood by all...

include (CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG (-pipe WBCRUN_CXX_FLAG_pipe)
IF (WBCRUN_CXX_FLAG_pipe)
  ADD_DEFINITIONS (-pipe)
ENDIF (WBCRUN_CXX_FLAG_pipe)

check_cxx_compiler_flag (-Wall WBCRUN_CXX_FLAG_Wall)
if (WBCRUN_CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (WBCRUN_CXX_FLAG_Wall)

IF (NOT BUILD_SHARED_LIBS)
   SET (BUILD_SHARED_LIBS True)
ENDIF (NOT BUILD_SHARED_LIBS)

if (NOT CMAKE_BUILD_TYPE)
   set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
   check_cxx_compiler_flag (-O0 WBCRUN_CXX_FLAG_O0)
   if (WBCRUN_CXX_FLAG_O0)
     add_definitions (-O0)
   endif (WBCRUN_CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")


### find dependencies

message ("WBCNET_DIR is \"${WBCNET_DIR}\"")
if (WBCNET_DIR)
  message ("using WBCNET_DIR")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${WBCNET_DIR}/include ${WBCNET_DIR} /usr/local/include)
  include_directories (${WBCNET_DIR}/include ${WBCNET_DIR} /usr/local/include)
  link_directories (${WBCNET_DIR}/lib ${WBCNET_DIR} /usr/local/lib)
else (WBCNET_DIR)
  message ("no WBCNET_DIR")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${WBCRUN_BINARY_DIR} /usr/local/include)
  include_directories (${WBCRUN_BINARY_DIR} /usr/local/include)
  link_directories (/usr/local/lib)
endif (WBCNET_DIR)

include (CheckIncludeFileCXX)

check_include_file_cxx (wbcnet/proxy.hpp WBCROS_HAVE_WBCNET)
if (${WBCROS_HAVE_WBCNET})
  message ("Great, found wbcnet headers.")
else (${WBCROS_HAVE_WBCNET})
  #  message (FATAL_ERROR "wbcnet not found, use -DWBCNET_DIR:path=/where/ever")
  message ("WARNING wbcnet not found")
  message ("if compilation fails, use -DWBCNET_DIR:path=/where/ever")
endif (${WBCROS_HAVE_WBCNET})

check_include_file_cxx (mqueue.h WBCRUN_HAVE_MQUEUE)
if (${WBCRUN_HAVE_MQUEUE})
  add_definitions (-DWBCRUN_HAVE_MQUEUE)
endif (${WBCRUN_HAVE_MQUEUE})

check_include_file_cxx (curses.h WBCRUN_HAVE_CURSES)
if (${WBCRUN_HAVE_CURSES})
  message ("Great, found curses.")
  list (APPEND WBCRUN_LIBS curses)
  add_definitions (-DWBCRUN_HAVE_CURSES)
else (${WBCRUN_HAVE_CURSES})
  message ("WARNING curses not found")
endif (${WBCRUN_HAVE_CURSES})

### do we have XmlRpc++ ?

# try to get XMLRPC_DIR from CMake or environment
# 1. CMake variable XMLRPC_DIR
# 2. environment variable XMLRPC_DIR
# 3. CMake variable WBC_BINDEPS_PATH
# 4. environment variable WBC_BINDEPS_PATH
# 5. give up
if (NOT XMLRPC_DIR)
  set (XMLRPC_DIR $ENV{XMLRPC_DIR})
  if (XMLRPC_DIR)
    message ("using XMLRPC_DIR from environment: ${XMLRPC_DIR}")
  else (XMLRPC_DIR)
    if (NOT WBC_BINDEPS_PATH)
      set (WBC_BINDEPS_PATH $ENV{WBC_BINDEPS_PATH})
    endif (NOT WBC_BINDEPS_PATH)
    if (NOT WBC_BINDEPS_PATH)
      message ("no WBC_BINDEPS_PATH, cannot infer XMLRPC_DIR")
    else (NOT WBC_BINDEPS_PATH)
      message ("using XMLRPC_DIR=WBC_BINDEPS_PATH: ${WBC_BINDEPS_PATH}")
      set (XMLRPC_DIR ${WBC_BINDEPS_PATH})
    endif (NOT WBC_BINDEPS_PATH)
  endif (XMLRPC_DIR)
endif (NOT XMLRPC_DIR)
if (XMLRPC_DIR)
  message ("using XMLRPC_DIR = ${XMLRPC_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${XMLRPC_DIR}/include ${XMLRPC_DIR})
  include_directories (${XMLRPC_DIR}/include ${XMLRPC_DIR})
  link_directories (${XMLRPC_DIR}/lib ${XMLRPC_DIR})
else (XMLRPC_DIR)
  message ("no XMLRPC_DIR (optional)")
endif (XMLRPC_DIR)

include (CheckIncludeFileCXX)
check_include_file_cxx (XmlRpc.h WBCRUN_HAVE_XMLRPC_HEADER)
if (${WBCRUN_HAVE_XMLRPC_HEADER})
  message ("Found XmlRpc++ headers")
  if (XMLRPC_DIR)
    message ("DBG --- find_library(... PATHS ${XMLRPC_DIR} ${XMLRPC_DIR}/lib)")
    find_library (WBCRUN_HAVE_XMLRPC_LIB XmlRpc PATHS ${XMLRPC_DIR} ${XMLRPC_DIR}/lib)
  else (XMLRPC_DIR)
    message ("DBG --- find_library(...) without PATHS")
    find_library (WBCRUN_HAVE_XMLRPC_LIB XmlRpc)
  endif (XMLRPC_DIR)
  message ("DBG --- WBCRUN_HAVE_XMLRPC_LIB = ${WBCRUN_HAVE_XMLRPC_LIB}")
  if (WBCRUN_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
    message (FATAL_ERROR "XmlRpc++ library not found, although the header XmlRpc.h was found")
  else (WBCRUN_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
    add_definitions (-DWBCRUN_HAVE_XMLRPC)
    list (APPEND WBCRUN_SRCS xmlrpc/directory.cpp)
    list (APPEND WBCRUN_LIBS XmlRpc)
  endif (WBCRUN_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
else (${WBCRUN_HAVE_XMLRPC_HEADER})
  message ("WARNING XmlRpc++ not found, or the XmlRpc.h header failed to compile")
  message ("  You can install it from the stanford-wbc 3rdparty directory")
  message ("  or from its origin on http://xmlrpcpp.sourceforge.net/ of course.")
  message ("  After installation, use the XMLRPC_DIR variable to point to the correct location.")
  message ("  You can either set it in the environment or using -DXMLRPC_DIR:path=/where/ever")
  message ("  on the cmake command line.")
endif (${WBCRUN_HAVE_XMLRPC_HEADER})

check_include_file_cxx (log4cxx/logger.h WBCRUN_HAVE_LOG4CXX)
if (WBCRUN_HAVE_LOG4CXX)
  message ("found log4cxx")
  add_definitions (-DHAVE_LOG4CXX)
  list (APPEND WBCRUN_LIBS log4cxx)
else (WBCRUN_HAVE_LOG4CXX)
  message ("WARNING did not find log4cxx, will use simplistic logging")
endif (WBCRUN_HAVE_LOG4CXX)


list (APPEND WBCRUN_LIBS wbcnet)

include_directories (${WBCRUN_BINARY_DIR})
add_library (wbcrun SHARED ${WBCRUN_SRCS})
target_link_libraries (wbcrun ${WBCRUN_LIBS})

add_custom_target (symlink-wbcrun
                   COMMAND ${WBCRUN_SOURCE_DIR}/lnwbcrun.sh ${WBCRUN_SOURCE_DIR}
                   COMMENT Creating symbolic link to source directory
		   VERBATIM)
add_dependencies (wbcrun symlink-wbcrun)

file (GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install (FILES ${headers} DESTINATION include/wbcrun)

file (GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.hpp")
install (FILES ${headers} DESTINATION include/wbcrun/msg)

install (TARGETS wbcrun
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

subdirs (test)
