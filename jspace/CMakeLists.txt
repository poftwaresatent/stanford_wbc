### included via subdir() command in top-level CMakeLists.txt

add_library (jspace SHARED
  State.cpp
  Model.cpp
  Status.cpp
  controller_library.cpp
  vector_util.cpp
  tao_dump.cpp
  tao_util.cpp
  )

###add_executable (example example.cpp)
###target_link_libraries (example jspace wbcnet Stanford_WBC)

FILE (GLOB headers "*.hpp")
INSTALL (FILES ${headers} DESTINATION include/jspace)

INSTALL (TARGETS jspace
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)
