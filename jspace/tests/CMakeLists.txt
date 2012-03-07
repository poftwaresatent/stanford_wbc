include_directories (
  ../../jspace
  ../../tao
  ../../wbc_tinyxml
  ../../3rdparty/gtest-1.6.0/include
  )

add_executable (testJspace testJspace.cpp)
target_link_libraries (testJspace jspace_test gtest pthread ${MAYBE_GCOV})
