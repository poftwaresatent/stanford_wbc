### included via subdir() command in top-level CMakeLists.txt

### do we have gtest ?

# try to get GTEST_DIR from CMake or environment
# 1. CMake variable GTEST_DIR
# 2. environment variable GTEST_DIR
# 3. CMake variable WBC_BINDEPS_PATH
# 4. environment variable WBC_BINDEPS_PATH
# 5. give up
if (NOT GTEST_DIR)
  set (GTEST_DIR $ENV{GTEST_DIR})
  if (GTEST_DIR)
    message ("using GTEST_DIR from environment: ${GTEST_DIR}")
  else (GTEST_DIR)
    if (NOT WBC_BINDEPS_PATH)
      set (WBC_BINDEPS_PATH $ENV{WBC_BINDEPS_PATH})
    endif (NOT WBC_BINDEPS_PATH)
    if (NOT WBC_BINDEPS_PATH)
      message ("no WBC_BINDEPS_PATH, cannot infer GTEST_DIR")
    else (NOT WBC_BINDEPS_PATH)
      message ("using GTEST_DIR=WBC_BINDEPS_PATH: ${WBC_BINDEPS_PATH}")
      set (GTEST_DIR ${WBC_BINDEPS_PATH})
    endif (NOT WBC_BINDEPS_PATH)
  endif (GTEST_DIR)
endif (NOT GTEST_DIR)
if (GTEST_DIR)
  message ("using GTEST_DIR = ${GTEST_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
  include_directories (${GTEST_DIR}/include ${GTEST_DIR})
  link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
else (GTEST_DIR)
  message ("no GTEST_DIR (optional)")
endif (GTEST_DIR)

include (CheckIncludeFileCXX)
check_include_file_cxx (gtest/gtest.h HAVE_GTEST_HEADER)
if (${HAVE_GTEST_HEADER})
  message ("Found gtest headers")
  if (GTEST_DIR)
    message ("DBG --- find_library(... PATHS ${GTEST_DIR} ${GTEST_DIR}/lib)")
    find_library (HAVE_GTEST_LIB gtest PATHS ${GTEST_DIR} ${GTEST_DIR}/lib)
  else (GTEST_DIR)
    message ("DBG --- find_library(...) without PATHS")
    find_library (HAVE_GTEST_LIB gtest)
  endif (GTEST_DIR)
  message ("DBG --- HAVE_GTEST_LIB = ${HAVE_GTEST_LIB}")
  if (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message (FATAL_ERROR "gtest library not found, although the header gtest/gtest.h was found")
  else (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message ("Found gtest library")
    add_executable (testBRBuilder testBRBuilder.cpp)
    target_link_libraries (testBRBuilder Stanford_WBC gtest)
    add_executable (testTAO testTAO.cpp)
    target_link_libraries (testTAO TAO_Dynamics_Engine gtest)
  endif (HAVE_GTEST_LIB MATCHES "NOTFOUND")
else (${HAVE_GTEST_HEADER})
  message ("WARNING gtest not found, or the gtest/gtest.h header failed to compile")
endif (${HAVE_GTEST_HEADER})

add_executable (testProcess testProcess.cpp)
target_link_libraries (testProcess Stanford_WBC)
