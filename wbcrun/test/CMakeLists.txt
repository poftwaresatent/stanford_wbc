list (APPEND TEST_LIBS wbcrun wbcnet)

if (${WBCRUN_HAVE_MQUEUE})
  list (APPEND TEST_LIBS rt)
endif (${WBCRUN_HAVE_MQUEUE})

if (${WBCRUN_HAVE_XMLRPC})
  message (STATUS "Adding tests for XmlRpc++ dependent code")
  add_executable (test_directory_server test_directory_server.cpp TestDirectory.cpp)
  target_link_libraries (test_directory_server ${TEST_LIBS} XmlRpc)
endif (${WBCRUN_HAVE_XMLRPC})
