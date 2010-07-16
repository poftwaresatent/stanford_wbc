cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (jspace_tests)

include_directories (
  .
  ../jspace
  ../saimatrix
  ../wbcnet
  ../tao
  ../wbc
  )

link_directories (${URDF_LIBDIRS})
#/wg/stor1a/roland/cturtle_karmic_wg_all/stacks/robot_model/urdf/lib;/wg/stor1a/roland/cturtle_karmic_wg_all/stacks/common/tinyxml;/wg/stor1a/roland/cturtle_karmic_wg_all/ros/core/roscpp/lib;/wg/stor1a/roland/cturtle_karmic_wg_all/ros/3rdparty/xmlrpcpp/lib;/wg/stor1a/roland/cturtle_karmic_wg_all/ros/core/rosconsole/lib;/wg/stor1a/roland/cturtle_karmic_wg_all/ros/core/roslib/lib;/wg/stor1a/roland/cturtle_karmic_wg_all/ros/tools/rospack/lib

add_library (jspace_tests model_library.cpp)
target_link_libraries (jspace_tests jspace Stanford_WBC)

add_executable (benchProxyDelays benchProxyDelays.cpp)
target_link_libraries (benchProxyDelays jspace_tests)

wbc_find_urdf ()

if (HAVE_GTEST)
  if (HAVE_ROS)
    add_executable (testJspace testJspace.cpp)
    target_link_libraries (testJspace jspace_tests gtest urdf)
  else (HAVE_ROS)
    message ("WARNING ROS not found")
  endif (HAVE_ROS)
else (HAVE_GTEST)
  message ("WARNING gtest not found")
endif (HAVE_GTEST)
