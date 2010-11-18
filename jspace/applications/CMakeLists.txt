include_directories (
  ../../jspace
  )

add_executable (sai2lotus sai2lotus.cpp)
target_link_libraries (sai2lotus jspace_test ${MAYBE_GCOV})

add_executable (trjcheck trjcheck.cpp)
target_link_libraries (trjcheck jspace_test ${MAYBE_GCOV})
