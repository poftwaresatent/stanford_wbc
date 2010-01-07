cmake_minimum_required (VERSION 2.4)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

project (wbc_tinyxml)
set (CMAKE_VERBOSE_MAKEFILE ON)

message (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
message ("configuring wbc_tinyxml")

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe wbc_tinyxml_CXX_FLAG_pipe)
if (wbc_tinyxml_CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (wbc_tinyxml_CXX_FLAG_pipe)

check_cxx_compiler_flag (-ansi wbc_tinyxml_CXX_FLAG_ansi)
if (wbc_tinyxml_CXX_FLAG_ansi)
  add_definitions (-ansi)
endif (wbc_tinyxml_CXX_FLAG_ansi)

check_cxx_compiler_flag (-pedantic wbc_tinyxml_CXX_FLAG_pedantic)
if (wbc_tinyxml_CXX_FLAG_pedantic)
  add_definitions (-pedantic)
endif (wbc_tinyxml_CXX_FLAG_pedantic)

check_cxx_compiler_flag (-Wall wbc_tinyxml_CXX_FLAG_Wall)
if (wbc_tinyxml_CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (wbc_tinyxml_CXX_FLAG_Wall)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set (wbc_tinyxml_DEBUG)
  check_cxx_compiler_flag (-O0 wbc_tinyxml_CXX_FLAG_O0)
  if (wbc_tinyxml_CXX_FLAG_O0)
    add_definitions (-O0)
  endif (wbc_tinyxml_CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

add_library (
  wbc_tinyxml SHARED
  wbc_tinystr.cpp
  wbc_tinyxml.cpp
  wbc_tinyxmlerror.cpp
  wbc_tinyxmlparser.cpp)

include_directories (${wbc_tinyxml_SOURCE_DIR})

install (FILES wbc_tinyxml.h wbc_tinystr.h DESTINATION include/wbc_tinyxml)
install (TARGETS wbc_tinyxml DESTINATION lib)
