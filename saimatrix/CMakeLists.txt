cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

project (SAI_Matrix)
set (CMAKE_VERBOSE_MAKEFILE ON)

message (">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
message ("configuring SAI_Matrix")

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

check_cxx_compiler_flag (-pipe SAI_Matrix_CXX_FLAG_pipe)
if (SAI_Matrix_CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (SAI_Matrix_CXX_FLAG_pipe)

check_cxx_compiler_flag (-Wall SAI_Matrix_CXX_FLAG_Wall)
if (SAI_Matrix_CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (SAI_Matrix_CXX_FLAG_Wall)

if (ANSI)
  check_cxx_compiler_flag (-ansi SAI_Matrix_CXX_FLAG_ansi)
  if (SAI_Matrix_CXX_FLAG_ansi)
    add_definitions (-ansi)
  endif (SAI_Matrix_CXX_FLAG_ansi)
  check_cxx_compiler_flag (-pedantic SAI_Matrix_CXX_FLAG_pedantic)
  if (SAI_Matrix_CXX_FLAG_pedantic)
    add_definitions (-pedantic)
  endif (SAI_Matrix_CXX_FLAG_pedantic)
endif (ANSI)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set (SAI_Matrix_DEBUG)
  check_cxx_compiler_flag (-O0 SAI_Matrix_CXX_FLAG_O0)
  if (SAI_Matrix_CXX_FLAG_O0)
    add_definitions (-O0)
  endif (SAI_Matrix_CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

### If we have LAPACK, we can enable the SVD and maybe other goodies ###

### unfortunately the following fails because it expects the project to
### enable FORTRAN, which requires a Fortran compiler, which we do not
### need...
# include (FindLAPACK)
# if (LAPACK_FOUND)
#  etc etc etc
# endif (LAPACK_FOUND)
### ...so, we just check for the libs individually

find_library (SAI_Matrix_FOUND_LAPACK lapack)
if (SAI_Matrix_FOUND_LAPACK)
  message ("found LAPACK")
  find_library (SAI_Matrix_FOUND_BLAS blas)
  if (SAI_Matrix_FOUND_BLAS)
    message ("found BLAS")
    set (SAI_Matrix_ENABLE_LAPACK True)
  else (SAI_Matrix_FOUND_BLAS)
    message (FATAL_ERROR "BLAS not found, please install it (e.g. `sudo apt-get install libblas-dev')")
  endif (SAI_Matrix_FOUND_BLAS)
else (SAI_Matrix_FOUND_LAPACK)
  message (FATAL_ERROR "LAPACK not found, please install it (e.g. `sudo apt-get install liblapack-dev')")
endif (SAI_Matrix_FOUND_LAPACK)

### The actual code ###

list (APPEND
  SAI_Matrix_SRCS
  SAITransform.cpp
  SAIMatrix.cpp
  SAIVector.cpp
  SAIMatrix3.cpp
  SAIVector3.cpp
  SAIMatrix6.cpp
  SAIVector6.cpp
  SAIQuaternion.cpp)

if (SAI_Matrix_ENABLE_LAPACK)
  list (APPEND SAI_Matrix_SRCS SAILapack.cpp)
  list (APPEND SAI_Matrix_LIBS lapack blas)
endif (SAI_Matrix_ENABLE_LAPACK)

add_library (SAI_Matrix SHARED ${SAI_Matrix_SRCS})
target_link_libraries (SAI_Matrix ${SAI_Matrix_LIBS})

if (SAI_Matrix_ENABLE_LAPACK)
  add_executable (test_SAILapack test_SAILapack.cpp)
  target_link_libraries (test_SAILapack SAI_Matrix)
endif (SAI_Matrix_ENABLE_LAPACK)

### A bit of a hack to keep headers alongside sources and still include <saimatrix/foo/bar.h> ###

add_custom_target (symlink-saimatrix
  COMMAND ${SAI_Matrix_SOURCE_DIR}/lnsaimatrix.sh ${SAI_Matrix_SOURCE_DIR}
  COMMENT Creating symbolic link to source directory
  WORKING_DIRECTORY ${SAI_Matrix_BINARY_DIR}
  VERBATIM)
add_dependencies (SAI_Matrix symlink-saimatrix)

include_directories (${SAI_Matrix_BINARY_DIR})

file (GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install (FILES ${headers} DESTINATION include/saimatrix)

install (TARGETS SAI_Matrix DESTINATION lib)
