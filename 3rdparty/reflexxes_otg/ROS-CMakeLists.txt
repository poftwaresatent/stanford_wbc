cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (ROS_BUILD_TYPE Release)

rosbuild_init ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_add_library (reflexxes_otg
  src/TypeIOTG.cpp
  src/TypeIOTGDecision.cpp
  src/TypeIOTGMath.cpp
  src/TypeIOTGPolynomial.cpp
  src/TypeIOTGProfiles.cpp
  )

#target_link_libraries(reflexxes_otg another_library)

rosbuild_add_executable (trjgen src/trjgen.cpp)
target_link_libraries (trjgen reflexxes_otg)
