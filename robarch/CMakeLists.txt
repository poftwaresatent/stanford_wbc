### included via subdir() command in top-level CMakeLists.txt

list (APPEND ROBARCH_SRC
  CRobotArchitect.cpp
  osimarchitect/parser/CCoordObj.cpp
  osimarchitect/parser/CJointNew.cpp
  osimarchitect/COsimArchitect.cpp
  osimarchitect/parser/CSkeletonLinkNew.cpp
  osimarchitect/parser/CSkeletonModelNew.cpp
  osimarchitect/parser/CTransformAxisObj.cpp
  osimarchitect/parser/CWrapCylinderNew.cpp
  osimarchitect/parser/CWrapEllipsoidNew.cpp
  osimarchitect/parser/SkeletonCoordinate.cpp
  osimarchitect/parser/SkeletonMarker.cpp
  osimarchitect/parser/CMuscleDefinition.cpp
  )

add_library (robarch SHARED ${ROBARCH_SRC})
