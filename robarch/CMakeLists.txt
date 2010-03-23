cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../macros.cmake)
wbc_init (Stanford_WBC_Robot_Architect)

list (APPEND ROBARCH_SRC
  robarch/CRobotArchitect.cpp
  robarch/osimarchitect/parser/CCoordObj.cpp
  robarch/osimarchitect/parser/CJointNew.cpp
  robarch/osimarchitect/COsimArchitect.cpp
  robarch/osimarchitect/parser/CSkeletonLinkNew.cpp
  robarch/osimarchitect/parser/CSkeletonModelNew.cpp
  robarch/osimarchitect/parser/CTransformAxisObj.cpp
  robarch/osimarchitect/parser/CWrapCylinderNew.cpp
  robarch/osimarchitect/parser/CWrapEllipsoidNew.cpp
  robarch/osimarchitect/parser/SkeletonCoordinate.cpp
  robarch/osimarchitect/parser/SkeletonMarker.cpp
  robarch/osimarchitect/parser/CMuscleDefinition.cpp
  robarch/rep_creators/CTaoRepCreator.cpp
  )

add_library (robarch SHARED ${ROBARCH_SRC})

include_directories (
  .
  ../saimatrix
  ../tao
  ../wbc_tinyxml
  ../wbcnet
  )

install (DIRECTORY robarch/
  DESTINATION include/robarch
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install (DIRECTORY robarch/
  DESTINATION include/robarch
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install (TARGETS robarch DESTINATION lib)
