cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (Stanford_WBC_Joint_Space_Dynamics)
wbc_find_urdf ()

if (NOT EIGEN2_DIR)
  set (EIGEN2_DIR $ENV{EIGEN2_DIR})
endif (NOT EIGEN2_DIR)
if (EIGEN2_DIR)
  message ("[JSPACE] EIGEN2_DIR is set to ${EIGEN2_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${EIGEN2_DIR}/include ${EIGEN2_DIR})
  include_directories (${EIGEN2_DIR}/include ${EIGEN2_DIR})
endif (EIGEN2_DIR)

check_include_file_cxx (eigen2/Eigen/Core HAVE_EIGEN2)
if (${HAVE_EIGEN2})
  message ("[JSPACE] found eigen2 headers")
else (${HAVE_EIGEN2})
  message (FATAL_ERROR "[JSPACE] Eigen2 library not found. Please install it e.g. using `sudo apt-get install libeigen2-dev' or from http://eigen.tuxfamily.org/ and tell me where to find it by passing the -DEIGEN2_DIR:path=/path/to/eigen2")
endif (${HAVE_EIGEN2})

list (APPEND SRCS
  jspace/State.cpp
  jspace/Model.cpp
  jspace/Status.cpp
  jspace/Controller.cpp
  jspace/controller_library.cpp
  jspace/vector_util.cpp
  jspace/tao_dump.cpp
  jspace/tao_util.cpp
  jspace/proxy_util.cpp
  jspace/servo_proxy.cpp
  jspace/robot_proxy.cpp
  )

if (HAVE_ROS)
  message ("[jspace] adding ROS and URDF support")
  list (APPEND SRCS
    jspace/ros/urdf_to_tao.cpp
    jspace/ros/urdf_dump.cpp
    jspace/ros/Model.cpp
    )
endif (HAVE_ROS)

include_directories (
  .
  ../tao
  ../wbcnet
  )

add_library (jspace SHARED ${SRCS})
if (HAVE_ROS)
  target_link_libraries (jspace wbcnet TAO_Dynamics_Engine urdf)
  # Some guesswork to make it easier for users of libjspace to handle
  # the optional ROS dependency.
  #  set_target_properties (jspace
  #    PROPERTIES
  #    INSTALL_RPATH ${URDF_LIBDIRS}
  #    BUILD_WITH_INSTALL_RPATH True)
else (HAVE_ROS)
  target_link_libraries (jspace wbcnet TAO_Dynamics_Engine)
endif (HAVE_ROS)

if (HAVE_GTEST)
  add_executable (testServoProxy tests/testServoProxy.cpp)
  target_link_libraries (testServoProxy jspace gtest)
endif (HAVE_GTEST)

file (GLOB headers "jspace/*.hpp")
install (FILES ${headers} DESTINATION include/jspace)

if (HAVE_ROS)
  file (GLOB ros_headers "jspace/ros/*.hpp")
  install (FILES ${ros_headers} DESTINATION include/jspace/ros)
endif (HAVE_ROS)

install (TARGETS jspace
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
