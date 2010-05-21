### included via subdir() command in top-level CMakeLists.txt

wbc_add_executable (wbcuser wbcuser.cpp)
wbc_add_executable (wbcmodel wbcmodel.cpp)
wbc_add_executable (wbcservo wbcservo.cpp)
wbc_add_executable (rawservo rawservo.cpp)
wbc_add_executable (jspace-servo jspace-servo.cpp)
wbc_add_executable (robot-bridge robot-bridge.cpp)

install (TARGETS wbcuser wbcmodel wbcservo rawservo jspace-servo robot-bridge DESTINATION bin)
