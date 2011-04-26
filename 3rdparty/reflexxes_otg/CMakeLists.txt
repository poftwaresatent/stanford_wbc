cmake_minimum_required (VERSION 2.4.6)

# not so trivial... # if (EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# not so trivial... #   message ("*** enabling ROS support")
# not so trivial... #   project (reflexxes_otg)
# not so trivial... #   include (ROS-CMakeLists.txt)
# not so trivial... # else (EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# not so trivial... #   message ("--- did not find ROS, using subproject CMake setup")
  include_directories (include)
  add_library (reflexxes_otg SHARED
    src/TypeIOTG.cpp
    src/TypeIOTGDecision.cpp
    src/TypeIOTGMath.cpp
    src/TypeIOTGPolynomial.cpp
    src/TypeIOTGProfiles.cpp
    )
  add_executable (trjgen src/trjgen.cpp)
  target_link_libraries (trjgen reflexxes_otg)
# not so trivial... # endif (EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
