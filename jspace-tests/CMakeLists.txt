cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (jspace_tests)

wbc_find_urdf ()

if (HAVE_GTEST)
  if (HAVE_ROS)
    include_directories (
      .
      ../jspace
      ../saimatrix
      ../wbcnet
      ../tao
      ../wbc
      )
    add_executable (testJspace testJspace.cpp)
    target_link_libraries (testJspace jspace Stanford_WBC gtest urdf)
  else (HAVE_ROS)
    message ("WARNING ROS not found")
  endif (HAVE_ROS)
else (HAVE_GTEST)
  message ("WARNING gtest not found")
endif (HAVE_GTEST)
