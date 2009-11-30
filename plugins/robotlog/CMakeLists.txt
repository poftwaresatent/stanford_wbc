cmake_minimum_required (VERSION 2.6)


# Well... this entire block serves to pull-in the wbc.cmake macro
# definitions. But we want to support separate installations as well
# as devkit-style setups. So we have to look in two places, depending
# on circumstances.
if (NOT WBC_ROOT)
  set (WBC_ROOT $ENV{WBC_ROOT})
endif (NOT WBC_ROOT)
if (WBC_ROOT)
  if (NOT EXISTS ${WBC_ROOT}/share/wbc.cmake)
    message ("[WBC] hmm... looks like a devkit-style build setup...")
    if (NOT Stanford_WBC_SOURCE_DIR)
      message (FATAL_ERROR "The Stanford_WBC_SOURCE_DIR variable is not set, I don't know where to look for the wbc.cmake file!.")
    endif (NOT Stanford_WBC_SOURCE_DIR)
    if (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
      include (${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
    else (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
      message (FATAL_ERROR "Neither ${WBC_ROOT}/share/wbc.cmake nor ${Stanford_WBC_SOURCE_DIR}/wbc.cmake exist, giving up.")
    endif (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
  else (NOT EXISTS ${WBC_ROOT}/share/wbc.cmake)
    include (${WBC_ROOT}/share/wbc.cmake)
  endif (NOT EXISTS ${WBC_ROOT}/share/wbc.cmake)
else (WBC_ROOT)
  message (FATAL_ERROR "WBC_ROOT must be set either in the environment or as a CMake variable. Follow the instructions in the README and have a look at the installation instructions on http://stanford-wbc.sourceforge.net/")
endif (WBC_ROOT)

list (APPEND LIBS expat Stanford_WBC)

wbc_add_plugin (robotlog plugin.cpp readlog.cpp writelog.cpp)
target_link_libraries (robotlog ${LIBS})

include (CheckIncludeFileCXX)

if (NOT GTEST_DIR)
  message (STATUS "environment GTEST_DIR = $ENV{GTEST_DIR}")
  set (GTEST_DIR $ENV{GTEST_DIR})
endif (NOT GTEST_DIR)

if (GTEST_DIR)
  message (STATUS "using GTEST_DIR ${GTEST_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
  include_directories (${GTEST_DIR}/include ${GTEST_DIR})
  link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
else (GTEST_DIR)
  message (STATUS "no GTEST_DIR")
endif (GTEST_DIR)

check_include_file_cxx (log4cxx/logger.h HAVE_LOG4CXX)
if (HAVE_LOG4CXX)
  add_definitions (-DHAVE_LOG4CXX)
endif (HAVE_LOG4CXX)

check_include_file_cxx (gtest/gtest.h HAVE_GTEST)
if (${HAVE_GTEST})
  add_executable (test test.cpp readlog.cpp writelog.cpp)
  target_link_libraries (test gtest ${LIBS})
else (${HAVE_GTEST})
  message (STATUS "WARNING gtest not found, use -DGTEST_DIR:path=/foo/bar to")
endif (${HAVE_GTEST})
