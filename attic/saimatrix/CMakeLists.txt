cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (SAI_Matrix)

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
  # Well, here we simply always fail if lapack is not found, so
  # actually we do not need the handling of "optional" stuff.
  message (FATAL_ERROR "LAPACK not found, please install it (e.g. `sudo apt-get install liblapack-dev')")
endif (SAI_Matrix_FOUND_LAPACK)

### The actual code ###

list (APPEND
  SAI_Matrix_SRCS
  saimatrix/SAITransform.cpp
  saimatrix/SAIMatrix.cpp
  saimatrix/SAIVector.cpp
  saimatrix/SAIMatrix3.cpp
  saimatrix/SAIVector3.cpp
  saimatrix/SAIMatrix6.cpp
  saimatrix/SAIVector6.cpp
  saimatrix/SAIQuaternion.cpp)

if (SAI_Matrix_ENABLE_LAPACK)
  list (APPEND SAI_Matrix_SRCS saimatrix/SAILapack.cpp)
  list (APPEND SAI_Matrix_LIBS lapack blas)
endif (SAI_Matrix_ENABLE_LAPACK)

add_library (SAI_Matrix SHARED ${SAI_Matrix_SRCS})
target_link_libraries (SAI_Matrix ${SAI_Matrix_LIBS} ${MAYBE_GCOV})

if (SAI_Matrix_ENABLE_LAPACK)
  add_executable (test_SAILapack tests/test_SAILapack.cpp)
  target_link_libraries (test_SAILapack SAI_Matrix ${MAYBE_GCOV})
endif (SAI_Matrix_ENABLE_LAPACK)

include_directories (${SAI_Matrix_SOURCE_DIR})

file (GLOB headers "${SAI_Matrix_SOURCE_DIR}/saimatrix/*.h")
install (FILES ${headers} DESTINATION include/saimatrix)

install (TARGETS SAI_Matrix DESTINATION lib)
