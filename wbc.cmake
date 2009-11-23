cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)


##################################################
#
# wbc_getvars ()
#
# Check for necessary variables and set them if possible.
# - WBC_ROOT has to be set before calling this macro
# - WBC_PLUGIN_PATH can be set, or it will be filled by this macro
# - WBC_BINDEPS_PATH, GTEST_DIR, XMLRPC_DIR, LOG4CXX_DIR: if not set,
#   will be attempted to be taken from the environment. As a side
#   effect, include and link directories will be adjusted to find
#   gtest, xmlrpc++, and log4cxx headers and libraries (either from
#   the specific variables, or summarily from the bindeps path). Note
#   that log4cxx will be taken from the ROS_BINDEPS_PATH environment
#   variable, if that is set, in order to avoid conflicts when we
#   build WBC in conjunction with ROS.
#
macro (wbc_getvars)
  # try to get WBC_ROOT from CMake or environment
  if (NOT WBC_ROOT)
    set (WBC_ROOT $ENV{WBC_ROOT})
    if (NOT WBC_ROOT)
      message (FATAL_ERROR "[WBC] WBC_ROOT is not set (neither in CMake nor in environment)")
    endif (NOT WBC_ROOT)
    message ("[WBC] using WBC_ROOT from environment: ${WBC_ROOT}")
  endif (NOT WBC_ROOT)
  
  # try to get WBC_PLUGIN_PATH from CMake or environment, fall back on ${WBC_ROOT}/plugins
  if (NOT WBC_PLUGIN_PATH)
    set (WBC_PLUGIN_PATH $ENV{WBC_PLUGIN_PATH})
    if (NOT WBC_PLUGIN_PATH)
      set (WBC_PLUGIN_PATH ${WBC_ROOT}/plugins)
      message ("[WBC] using fallback WBC_PLUGIN_PATH: ${WBC_PLUGIN_PATH}")
    else (NOT WBC_PLUGIN_PATH)
      message ("[WBC] using WBC_PLUGIN_PATH from environment: ${WBC_PLUGIN_PATH}")
    endif (NOT WBC_PLUGIN_PATH)
  endif (NOT WBC_PLUGIN_PATH)
  
  # try to get GTEST_DIR from CMake or environment
  if (NOT GTEST_DIR)
    set (GTEST_DIR $ENV{GTEST_DIR})
    if (GTEST_DIR)
      message ("[WBC] using GTEST_DIR from environment: ${GTEST_DIR}")
    endif (GTEST_DIR)
  endif (NOT GTEST_DIR)
  if (GTEST_DIR)
    list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
    include_directories (${GTEST_DIR}/include ${GTEST_DIR})
    link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
  endif (GTEST_DIR)
  
  # try to get XMLRPC_DIR from CMake or environment
  if (NOT XMLRPC_DIR)
    set (XMLRPC_DIR $ENV{XMLRPC_DIR})
    if (XMLRPC_DIR)
      message ("[WBC] using XMLRPC_DIR from environment: ${XMLRPC_DIR}")
    endif (XMLRPC_DIR)
  endif (NOT XMLRPC_DIR)
  if (XMLRPC_DIR)
    list (APPEND CMAKE_REQUIRED_INCLUDES ${XMLRPC_DIR}/include ${XMLRPC_DIR})
    include_directories (${XMLRPC_DIR}/include ${XMLRPC_DIR})
    link_directories (${XMLRPC_DIR}/lib ${XMLRPC_DIR})
  endif (XMLRPC_DIR)
  
  # try to get LOG4CXX_DIR from CMake or environment
  if (NOT LOG4CXX_DIR)
    set (LOG4CXX_DIR $ENV{LOG4CXX_DIR})
  endif (NOT LOG4CXX_DIR)
  if (NOT LOG4CXX_DIR)
    set (LOG4CXX_DIR $ENV{ROS_BINDEPS_PATH})
  endif (NOT LOG4CXX_DIR)
  if (NOT LOG4CXX_DIR)
    set (LOG4CXX_DIR ${WBC_BINDEPS_PATH})
  endif (NOT LOG4CXX_DIR)
  if (NOT LOG4CXX_DIR)
    set (LOG4CXX_DIR $ENV{WBC_BINDEPS_PATH})
  endif (NOT LOG4CXX_DIR)
  if (LOG4CXX_DIR)
    list (APPEND CMAKE_REQUIRED_INCLUDES ${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
    include_directories (${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
    link_directories (${LOG4CXX_DIR}/lib ${LOG4CXX_DIR})
  endif (LOG4CXX_DIR)
  message ("[WBC] debug -- LOG4CXX_DIR is ${LOG4CXX_DIR}")
  
  # try to get WBC_BINDEPS_PATH from CMake or environment
  if (NOT WBC_BINDEPS_PATH)
    set (WBC_BINDEPS_PATH $ENV{WBC_BINDEPS_PATH})
    if (WBC_BINDEPS_PATH)
      message ("[WBC] using WBC_BINDEPS_PATH from environment: ${WBC_BINDEPS_PATH}")
    endif (WBC_BINDEPS_PATH)
  endif (NOT WBC_BINDEPS_PATH)
  if (WBC_BINDEPS_PATH)
    list (APPEND CMAKE_REQUIRED_INCLUDES ${WBC_BINDEPS_PATH}/include ${WBC_BINDEPS_PATH})
    include_directories (${WBC_BINDEPS_PATH}/include ${WBC_BINDEPS_PATH})
    link_directories (${WBC_BINDEPS_PATH}/lib ${WBC_BINDEPS_PATH})
  endif (WBC_BINDEPS_PATH)
endmacro (wbc_getvars)


##################################################
#
# wbc_init (project_name)
#
# Set up some sensible flags and variables. Calls CMake's project()
# command for you, using the provided project_name parameter.
# - CMAKE_VERBOSE_MAKEFILE will be switched on
# - BUILD_SHARED_LIBS will be switched on, unless disabled before
# - CMAKE_BUILD_TYPE will be set to "Debug" unless defined to something else before
# - operating-system flag -DWIN32, -DOSX, -DLINUX, or -DOPENBSD will be set
# - compiler flags -pipe and -Wall will be set
# - compiler flag -O0 will be set for debug build
#
macro (wbc_init PROJECT_NAME)
  message ("[WBC] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
  message ("[WBC] BEGIN base config of ${PROJECT_NAME}")
  
  project (${PROJECT_NAME})
  
  # nothing worse than not knowing why a header could not be found, so
  # we wanna see those -I directives etc
  set (CMAKE_VERBOSE_MAKEFILE ON)
  if (NOT BUILD_SHARED_LIBS)
    set (BUILD_SHARED_LIBS True)
  endif (NOT BUILD_SHARED_LIBS)
  
  # some things might depend on the operating system
  if (WIN32)
    message ("[WBC] Detected Microsoft Windows")
    add_definitions (-DWIN32)
  else (WIN32)
    if (APPLE)
      message ("[WBC] Detected Mac OS X")
      add_definitions (-DOSX)
      # add the default macports location to include and link path
      include_directories (/opt/local/include)
      link_directories (/opt/local/lib)
    endif (APPLE)
    if (CMAKE_SYSTEM_NAME MATCHES Linux)
      message ("[WBC] Detected Linux")
      add_definitions (-DLINUX)
    endif (CMAKE_SYSTEM_NAME MATCHES Linux)
    if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
      message ("[WBC] Detected OpenBSD")
      add_definitions (-DOPENBSD)
      include_directories (/usr/local/include)
      link_directories (/usr/local/lib)
    endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
  endif(WIN32)
  
  # -pipe and -Wall are pretty sensible default flags for GCC
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
  if (CXX_FLAG_pipe)
    add_definitions (-pipe)
  endif (CXX_FLAG_pipe)
  check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
  if (CXX_FLAG_Wall)
    add_definitions (-Wall)
  endif (CXX_FLAG_Wall)
  
  # explicitly default to debug build and set -O0 in that case
  if (NOT CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Debug)
  endif (NOT CMAKE_BUILD_TYPE)
  if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
    if (CXX_FLAG_O0)
      add_definitions (-O0)
    endif (CXX_FLAG_O0)
  endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  
  # make sure we can find our headers
  wbc_getvars()
  include_directories (${WBC_ROOT}/include)
    
  # use decent logging if available
  include (CheckIncludeFileCXX)
  check_include_file_cxx (log4cxx/logger.h HAVE_LOG4CXX)
  if (HAVE_LOG4CXX)
    add_definitions (-DHAVE_LOG4CXX)
    list (APPEND LIBS log4cxx)
  else (HAVE_LOG4CXX)
    message ("[WBC] WARNING did not find log4cxx, will use simplistic logging")
  endif (HAVE_LOG4CXX)

  # use curses (for key codes) if available
  check_include_file_cxx (curses.h HAVE_CURSES)
  if (${HAVE_CURSES})
    add_definitions (-DHAVE_CURSES)
  else (${HAVE_CURSES})
    message ("[WBC] WARNING did not find curses, key codes will not be available")
  endif (${HAVE_CURSES})
  
  message ("[WBC] FINISHED base config of ${PROJECT_NAME}")
  message ("[WBC] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
endmacro (wbc_init)


##################################################
#
# wbc_add_plugin (name source1 source2 ... sourceN)
#
# Add a plugin to the whole-body controller. This is basically just a
# wrapper around CMake's standard add_library() command. The plugin
# file will be named according to the CMake rules concerning the given
# name, and the sources passed to the wbc_add_plugin() command will be
# compiled and linked into the plugin.
#
macro (wbc_add_plugin PLUGIN_NAME)
  message ("[WBC] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
  message ("[WBC] BEGIN configuring plugin ${PLUGIN_NAME}")
  
  wbc_init (${PLUGIN_NAME})
  
  add_library (${PLUGIN_NAME} MODULE ${ARGN})
  set_target_properties (${PLUGIN_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${WBC_PLUGIN_PATH})
  
  message ("[WBC] FINISHED configuring WBC plugin ${PLUGIN_NAME}")
  message ("[WBC] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
endmacro (wbc_add_plugin)
