include_directories (
  ../../jspace
  ../../wbcnet
  ../../tao
  ../../wbc_tinyxml
  )

add_library (jspace_tests util.cpp model_library.cpp sai_brep.cpp sai_brep_parser.cpp)
target_link_libraries (jspace_tests jspace wbc_tinyxml ${MAYBE_GCOV})

add_executable (benchProxyDelays benchProxyDelays.cpp)
target_link_libraries (benchProxyDelays jspace_tests ${MAYBE_GCOV})

if (HAVE_GTEST)
  
  add_executable (testServoProxy testServoProxy.cpp)
  target_link_libraries (testServoProxy jspace gtest ${MAYBE_GCOV})
  
  add_executable (testJspace testJspace.cpp)
  target_link_libraries (testJspace jspace_tests gtest ${MAYBE_GCOV})
  
  if (HAVE_URDF)
    # seems somewhat redundant, but did not work without
    link_directories (${URDF_LIBDIRS})
    add_executable (testJspaceUrdf testJspaceUrdf.cpp)
    target_link_libraries (testJspaceUrdf jspace_tests gtest urdf ${MAYBE_GCOV})
  else (HAVE_URDF)
    message ("WARNING URDF not found")
  endif (HAVE_URDF)

else (HAVE_GTEST)
  message ("WARNING gtest not found")
endif (HAVE_GTEST)
