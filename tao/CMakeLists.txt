cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

project (TAO_Dynamics_Engine)
set (CMAKE_VERBOSE_MAKEFILE ON)

message (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
message ("configuring TAO_Dynamics_Engine")

if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)

### Set up some system-dependent things... ###

message ("System: ${CMAKE_SYSTEM_NAME}")

if (WIN32)
  message ("Detected Microsoft Windows")
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    message ("Detected Mac OS X")
    add_definitions (-DOSX)
    # add the default macports location to include and link path
    include_directories (/opt/local/include)
    link_directories (/opt/local/lib)
  endif (APPLE)
  
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    message ("Detected Linux")
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    message ("Detected OpenBSD")
    add_definitions (-DOPENBSD)
    # OpenBSD will not pick up /usr/local on its own
    include_directories (/usr/local/include)
    link_directories (/usr/local/lib)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif (WIN32)

### Some compiler flags are not understood by all...

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe TAO_Dynamics_Engine_CXX_FLAG_pipe)
if (TAO_Dynamics_Engine_CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (TAO_Dynamics_Engine_CXX_FLAG_pipe)

check_cxx_compiler_flag (-Wall TAO_Dynamics_Engine_CXX_FLAG_Wall)
if (TAO_Dynamics_Engine_CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (TAO_Dynamics_Engine_CXX_FLAG_Wall)

if (ANSI)
  check_cxx_compiler_flag (-ansi TAO_Dynamics_Engine_CXX_FLAG_ansi)
  if (TAO_Dynamics_Engine_CXX_FLAG_ansi)
    add_definitions (-ansi)
  endif (TAO_Dynamics_Engine_CXX_FLAG_ansi)
  check_cxx_compiler_flag (-pedantic TAO_Dynamics_Engine_CXX_FLAG_pedantic)
  if (TAO_Dynamics_Engine_CXX_FLAG_pedantic)
    add_definitions (-pedantic)
  endif (TAO_Dynamics_Engine_CXX_FLAG_pedantic)
endif (ANSI)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set (TAO_Dynamics_Engine_DEBUG)
  check_cxx_compiler_flag (-O0 TAO_Dynamics_Engine_CXX_FLAG_O0)
  if (TAO_Dynamics_Engine_CXX_FLAG_O0)
    add_definitions (-O0)
  endif (TAO_Dynamics_Engine_CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

### The actual code ###

add_library (
  TAO_Dynamics_Engine SHARED
  dynamics/taoCNode.cpp
  dynamics/taoABJoint.cpp
  dynamics/taoNode.cpp
  dynamics/taoABNode.cpp
  dynamics/taoWorld.cpp
  dynamics/taoJoint.cpp
  dynamics/taoABDynamics.cpp
  dynamics/taoGroup.cpp
  dynamics/taoDynamics.cpp
  matrix/TaoDeMatrix6.cpp
  matrix/TaoDeVector6.cpp
  matrix/TaoDeQuaternionf.cpp
  matrix/TaoDeMatrix3f.cpp
  matrix/TaoDeTransform.cpp
  utility/TaoDeMassProp.cpp
  utility/TaoDeLogger.cpp)

### A bit of a hack to keep headers alongside sources and still include <tao/foo/bar.h> ###

add_custom_target (symlink-tao
  COMMAND ${TAO_Dynamics_Engine_SOURCE_DIR}/lntao.sh ${TAO_Dynamics_Engine_SOURCE_DIR}
  COMMENT Creating symbolic link to source directory
  WORKING_DIRECTORY ${TAO_Dynamics_Engine_BINARY_DIR}
  VERBATIM)
add_dependencies (TAO_Dynamics_Engine symlink-tao)

include_directories (${TAO_Dynamics_Engine_BINARY_DIR})

install (DIRECTORY ./
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install (DIRECTORY ./
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

install (TARGETS TAO_Dynamics_Engine DESTINATION lib)
