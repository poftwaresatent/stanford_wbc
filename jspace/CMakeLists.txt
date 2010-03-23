### included via subdir() command in top-level CMakeLists.txt

list (APPEND SRCS
  State.cpp
  Model.cpp
  Status.cpp
  controller_library.cpp
  vector_util.cpp
  tao_dump.cpp
  tao_util.cpp
  )

if (HAVE_ROS)
  message ("[jspace] adding ROS and URDF support")
  list (APPEND SRCS
    ros/urdf_to_tao.cpp
    ros/urdf_dump.cpp
    ros/Model.cpp
    )
endif (HAVE_ROS)

add_library (jspace SHARED ${SRCS})

file (GLOB headers "*.hpp")
install (FILES ${headers} DESTINATION include/jspace)

if (HAVE_ROS)
  file (GLOB ros_headers "ros/*.hpp")
  install (FILES ${ros_headers} DESTINATION include/jspace/ros)
endif (HAVE_ROS)

install (TARGETS jspace
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
