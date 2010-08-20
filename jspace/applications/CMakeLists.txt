include_directories (
  ../../jspace
#  ../../wbcnet
#  ../../tao
#  ../../wbc_tinyxml
  )

add_executable (sai2lotus sai2lotus.cpp)
target_link_libraries (sai2lotus jspace_test ${MAYBE_GCOV})
