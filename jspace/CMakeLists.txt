list (APPEND SRCS
  jspace/State.cpp
  jspace/Model.cpp
  jspace/Status.cpp
  jspace/Controller.cpp
  jspace/controller_library.cpp
  jspace/vector_util.cpp
  jspace/tao_dump.cpp
  jspace/tao_util.cpp
  jspace/inertia_util.cpp
  jspace/wrap_eigen.cpp
  jspace/strutil.cpp
  )

include_directories (
  .
  ../tao
  ../wbc_tinyxml
  )

add_library (jspace SHARED ${SRCS})
target_link_libraries (jspace tao-de ${MAYBE_GCOV})

add_library (jspace_test SHARED
  jspace/test/util.cpp
  jspace/test/model_library.cpp
  jspace/test/sai_brep.cpp
  jspace/test/sai_brep_parser.cpp
  )
target_link_libraries (jspace_test jspace wbc_tinyxml ${MAYBE_GCOV})

file (GLOB headers "jspace/*.hpp")
install (FILES ${headers} DESTINATION include/jspace)

file (GLOB test_headers "jspace/test/*.hpp")
install (FILES ${test_headers} DESTINATION include/jspace/test)

install (TARGETS jspace jspace_test
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

subdirs (tests applications)
