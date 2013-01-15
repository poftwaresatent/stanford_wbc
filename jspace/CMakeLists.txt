list (APPEND SRCS
  jspace/State.cpp
  jspace/Model.cpp
  jspace/Status.cpp
  jspace/Controller.cpp
  jspace/controller_library.cpp
  jspace/vector_util.cpp
### temporarily (?) moved to tests/.
#  jspace/inertia_util.cpp
  jspace/wrap_eigen.cpp
  jspace/strutil.cpp
  )

include_directories (
  .
  ../3rdparty/rbdl/src
  ${CMAKE_BINARY_DIR}/3rdparty/rbdl/src
  )

add_library (jspace SHARED ${SRCS})
target_link_libraries (jspace ${MAYBE_GCOV})

file (GLOB headers "jspace/*.hpp")
install (FILES ${headers} DESTINATION include/jspace)

install (TARGETS jspace
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

message ("*** REFACTORING for RBDL: jspace tests and applications are temporarily out of service")
#subdirs (tests applications)
