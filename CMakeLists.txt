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
      /opt/local/include/eigen2
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /opt/local/include
      /opt/local/include/eigen2
      )
    link_directories (/opt/local/lib)
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
    include_directories (/usr/include/eigen2)
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen2)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    include_directories (
      /usr/local/include
      /usr/local/include/eigen2
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen2)
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

# we should probably just hardcode this in our snapshot of tinyxml...
add_definitions (-DTIXML_USE_STL)

##################################################
# configure-time checks

include (CheckIncludeFileCXX)

# Eigen2 (required)

if (EIGEN2_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${EIGEN2_DIR}/include
    ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2
    ${EIGEN2_DIR}/eigen2
    )
  include_directories (
    ${EIGEN2_DIR}/include
    ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2
    ${EIGEN2_DIR}/
    eigen2
    )
endif (EIGEN2_DIR)
check_include_file_cxx (Eigen/Core HAVE_EIGEN2)
if (HAVE_EIGEN2)
  message ("*** found Eigen2 (required)")
else (HAVE_EIGEN2)
  message (FATAL_ERROR "Please install Eigen2, e.g. `sudo apt-get install libeigen2-dev' or from http://eigen.tuxfamily.org/ and pass -DEIGEN2_DIR:path=/path/to/eigen2 to cmake next time you call it.")
endif (HAVE_EIGEN2)

# gtest (optional)

if (GTEST_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
  include_directories (${GTEST_DIR}/include ${GTEST_DIR})
  link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
endif (GTEST_DIR)
check_include_file_cxx (gtest/gtest.h HAVE_GTEST_HEADER)
if (${HAVE_GTEST_HEADER})
  if (GTEST_DIR)
    find_library (HAVE_GTEST_LIB gtest PATHS ${GTEST_DIR} ${GTEST_DIR}/lib)
  else (GTEST_DIR)
    find_library (HAVE_GTEST_LIB gtest)
  endif (GTEST_DIR)
  if (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message (FATAL_ERROR "gtest library not found, although the header gtest/gtest.h was found")
  else (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message ("*** enabling gtest")
    set (HAVE_GTEST TRUE)
    add_definitions (-DHAVE_GTEST)
  endif (HAVE_GTEST_LIB MATCHES "NOTFOUND")
else (${HAVE_GTEST_HEADER})
  message ("--- gtest not found (optional)")
endif (${HAVE_GTEST_HEADER})

##################################################
### go into subdirs

subdirs (
  tao
  wbc_tinyxml
  jspace
  )
