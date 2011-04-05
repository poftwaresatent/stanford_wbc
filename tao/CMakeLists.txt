add_library (
  tao-de SHARED
  tao/dynamics/taoCNode.cpp
  tao/dynamics/taoABJoint.cpp
  tao/dynamics/taoNode.cpp
  tao/dynamics/taoABNode.cpp
  tao/dynamics/taoWorld.cpp
  tao/dynamics/taoJoint.cpp
  tao/dynamics/taoABDynamics.cpp
  tao/dynamics/taoGroup.cpp
  tao/dynamics/taoDynamics.cpp
  tao/matrix/TaoDeMatrix6.cpp
  tao/matrix/TaoDeVector6.cpp
  tao/matrix/TaoDeQuaternionf.cpp
  tao/matrix/TaoDeMatrix3f.cpp
  tao/matrix/TaoDeTransform.cpp
  tao/utility/TaoDeMassProp.cpp
  tao/utility/TaoDeLogger.cpp)

target_link_libraries (tao-de ${MAYBE_GCOV})

if (HAVE_GTEST)
  add_executable (testTAO tests/testTAO.cpp)
  target_link_libraries (testTAO tao-de gtest pthread ${MAYBE_GCOV})
endif (HAVE_GTEST)

include_directories (.)

install (DIRECTORY tao/
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install (DIRECTORY tao/
  DESTINATION include/tao
  FILES_MATCHING PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

install (TARGETS tao-de DESTINATION lib)
