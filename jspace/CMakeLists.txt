cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (Stanford_WBC_Joint_Space_Dynamics)
wbc_find_urdf ()

list (APPEND SRCS
  jspace/State.cpp
  jspace/Model.cpp
  jspace/Status.cpp
  jspace/controller_library.cpp
  jspace/vector_util.cpp
  jspace/tao_dump.cpp
  jspace/tao_util.cpp
  )

if (HAVE_ROS)
  message ("[jspace] adding ROS and URDF support")
  list (APPEND SRCS
    jspace/ros/urdf_to_tao.cpp
    jspace/ros/urdf_dump.cpp
    jspace/ros/Model.cpp
    )
endif (HAVE_ROS)

add_library (jspace SHARED ${SRCS})

### Would be nice, but we need to -Wl,-rpath,FOO to get it to work...
# target_link_libraries (jspace ${URDF_LIBS})

include_directories (
  .
  ../saimatrix
  ../tao
  ../wbcnet
  )

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
