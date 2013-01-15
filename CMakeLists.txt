cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)

##################################################
# basic setup

project (Stanford_WBC)
set (CMAKE_VERBOSE_MAKEFILE ON)
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)
if (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

###include_directories (.)

##################################################
# OS detection

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
    include_directories (
      /opt/local/include
      /opt/local/include/eigen3
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /opt/local/include
      /opt/local/include/eigen3
      )
    link_directories (/opt/local/lib)
#    # workaround for limited fltk-1.3 in macports, 2011-05-08
#    #    set (CMAKE_OSX_ARCHITECTURES "ppc;i386;ppc64;x86_64")
#    set (CMAKE_OSX_ARCHITECTURES "i386")
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
    include_directories (/usr/include/eigen3)
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen3)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    include_directories (
      /usr/local/include
      /usr/local/include/eigen3
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen3)
    link_directories (/usr/local/lib)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

##################################################
# compiler flags (other than header and library paths)

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
  if (CXX_FLAG_O0)
    add_definitions (-O0)
  endif (CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

##################################################
# configure-time checks

include (CheckIncludeFileCXX)

# Eigen3 (required)

if (EIGEN3_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${EIGEN3_DIR}/include
    ${EIGEN3_DIR}
    ${EIGEN3_DIR}/include/eigen3
    ${EIGEN3_DIR}/eigen3
    )
  include_directories (
    ${EIGEN3_DIR}/include
    ${EIGEN3_DIR}
    ${EIGEN3_DIR}/include/eigen3
    ${EIGEN3_DIR}/
    eigen3
    )
endif (EIGEN3_DIR)
check_include_file_cxx (Eigen/Core HAVE_EIGEN3)
if (HAVE_EIGEN3)
  message ("*** found Eigen3 (required)")
else (HAVE_EIGEN3)
  message (FATAL_ERROR "Please install Eigen3, e.g. `sudo apt-get install libeigen3-dev' or from http://eigen.tuxfamily.org/ and pass -DEIGEN3_DIR:path=/path/to/eigen3 to cmake next time you call it.")
endif (HAVE_EIGEN3)

# find Boost (required, just the smart pointers though)

if (BOOST_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
  include_directories (
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
endif (BOOST_DIR)
check_include_file_cxx (boost/shared_ptr.hpp HAVE_BOOST)
if (HAVE_BOOST)
  message ("*** found Boost (required)")
else (HAVE_BOOST)
  message (FATAL_ERROR "Please install the Boost smart_ptr library, and/or tell me where to find it by passing -DBOOST_DIR:path=/path/to/boost to cmake next time (you probably need to clear its cache first).")
endif (HAVE_BOOST)

# fltk (optional, for tutorials)

check_include_file_cxx (FL/Fl.H HAVE_FLTK)
if (HAVE_FLTK)
  message ("*** found fltk, enabling tutorials")
else (HAVE_FLTK)
  message ("--- fltk not found (optional)")
endif (HAVE_FLTK)

##################################################
### go into subdirs

subdirs (
  3rdparty/rbdl
  3rdparty/yaml-cpp
  3rdparty/reflexxes_otg
  3rdparty/gtest-1.6.0
  jspace
  opspace
  )

if (HAVE_FLTK)
  message ("*** REFACTORING for RBDL: tutorials temporarily out of service")
  ###  subdirs (tutorials)
endif (HAVE_FLTK)
