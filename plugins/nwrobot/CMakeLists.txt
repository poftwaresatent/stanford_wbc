cmake_minimum_required (VERSION 2.6)

include (../../wbc.cmake)

### this plugin only makes sense if we have libnetwrapper...
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
  list (APPEND CMAKE_REQUIRED_INCLUDES ${NETWRAP_DIR}/include)
  include_directories (${NETWRAP_DIR}/include)
  link_directories (${NETWRAP_DIR}/lib)
endif (NETWRAP_DIR)

check_include_file_cxx (NetWrapper.h HAVE_NETWRAP)
if (HAVE_NETWRAP)
  message ("found NetWrapper.h")
  add_definitions (-DHAVE_NETWRAP)
  wbc_add_plugin (nwrobot plugin.cpp RobotNetWrapper.cpp)
  target_link_libraries (nwrobot netwrapper)
else (HAVE_NETWRAP)
  message ("NetWrapper.h not found, cannot build the nwrobot plugin")
endif (HAVE_NETWRAP)
