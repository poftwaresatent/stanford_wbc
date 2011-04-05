include_directories (
  ../../jspace
  ../../tao
  ../../wbc_tinyxml
  )

if (HAVE_GTEST)
  
  add_executable (testJspace testJspace.cpp)
  target_link_libraries (testJspace jspace_test gtest pthread ${MAYBE_GCOV})

endif (HAVE_GTEST)
