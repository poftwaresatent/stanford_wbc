include_directories (
  ../../jspace
  )

add_executable (sai2lotus sai2lotus.cpp)
target_link_libraries (sai2lotus jspace_test ${MAYBE_GCOV})

add_executable (trjsim trjsim.cpp)
target_link_libraries (trjsim jspace_test ${MAYBE_GCOV})

add_executable (dynsim dynsim.cpp)
target_link_libraries (dynsim jspace_test ${MAYBE_GCOV})
