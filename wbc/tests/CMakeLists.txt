### included via subdir() command in top-level CMakeLists.txt

if (HAVE_GTEST)
  add_executable (testTAO testTAO.cpp)
  target_link_libraries (testTAO TAO_Dynamics_Engine gtest)
  add_executable (testJspace testJspace.cpp)
  target_link_libraries (testJspace jspace Stanford_WBC gtest)
else (HAVE_GTEST)
  message ("WARNING gtest not found, cannot build some (most) tests")
endif (HAVE_GTEST)

if (HAVE_XMLRPC)
  add_executable (testDirectoryServer testDirectoryServer.cpp TestDirectory.cpp)
  target_link_libraries (testDirectoryServer Stanford_WBC XmlRpc)
else (HAVE_XMLRPC)
  message ("WARNING XmlRpc++ not found, cannot build directory server test")
endif (HAVE_XMLRPC)

add_executable (testProcess testProcess.cpp)
target_link_libraries (testProcess Stanford_WBC)

add_executable (testBehaviorParser testBehaviorParser.cpp)
target_link_libraries (testBehaviorParser Stanford_WBC)

add_executable (testDtorCheck testDtorCheck.cpp)
target_link_libraries (testDtorCheck Stanford_WBC)
