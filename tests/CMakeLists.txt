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

### do we have xmlrpc++?

check_include_file_cxx (XmlRpc.h WBC_TESTS_HAVE_XMLRPC_HEADER)
if (${WBC_TESTS_HAVE_XMLRPC_HEADER})
  message ("Found XmlRpc++ headers")
  if (XMLRPC_DIR)
    find_library (WBC_TESTS_HAVE_XMLRPC_LIB XmlRpc PATHS ${XMLRPC_DIR} ${XMLRPC_DIR}/lib)
  else (XMLRPC_DIR)
    find_library (WBC_TESTS_HAVE_XMLRPC_LIB XmlRpc)
  endif (XMLRPC_DIR)
  if (WBC_TESTS_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
    message (FATAL_ERROR "XmlRpc++ library not found, although the header XmlRpc.h was found")
  else (WBC_TESTS_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
    add_definitions (-DHAVE_XMLRPC)
    add_executable (testDirectoryServer testDirectoryServer.cpp TestDirectory.cpp)
    # do we even need to add libXmlRpc again? does it hurt?
    target_link_libraries (testDirectoryServer Stanford_WBC XmlRpc)
  endif (WBC_TESTS_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
else (${WBC_TESTS_HAVE_XMLRPC_HEADER})
  message ("WARNING XmlRpc++ not found, or the XmlRpc.h header failed to compile")
  message ("  You can install it from the stanford-wbc 3rdparty directory")
  message ("  or from its origin on http://xmlrpcpp.sourceforge.net/ of course.")
  message ("  After installation, use the XMLRPC_DIR variable to point to the correct location.")
  message ("  You can either set it in the environment or using -DXMLRPC_DIR:path=/where/ever")
  message ("  on the cmake command line.")
endif (${WBC_TESTS_HAVE_XMLRPC_HEADER})

### tests that depend on nothing else...

add_executable (testProcess testProcess.cpp)
target_link_libraries (testProcess Stanford_WBC)
