### included via subdir() command in top-level CMakeLists.txt

add_executable (wbcuser wbcuser.cpp)
target_link_libraries (wbcuser Stanford_WBC)

add_executable (wbcmodel wbcmodel.cpp)
target_link_libraries (wbcmodel Stanford_WBC)

add_executable (wbcservo wbcservo.cpp)
target_link_libraries (wbcservo Stanford_WBC)

add_executable (rawservo rawservo.cpp)
target_link_libraries (rawservo Stanford_WBC)

set_target_properties(
  wbcuser wbcmodel wbcservo rawservo
  PROPERTIES
  INSTALL_RPATH ${WBC_ROOT}/lib:${WBC_ROOT}/bindeps/lib)

install (TARGETS wbcuser wbcmodel wbcservo rawservo DESTINATION bin)
