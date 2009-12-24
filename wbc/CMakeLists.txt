### included via subdir() command in top-level CMakeLists.txt

list (APPEND LIBS
  wbc_tinyxml
  SAI_Matrix
  TAO_Dynamics_Engine
  wbcnet
  wbcrun
  dl
  expat
  )

add_library (
  Stanford_WBC SHARED
  
  core/Plugin.cpp
  core/RobotFactory.cpp
  core/TaskModelBase.cpp
  core/MobileManipulatorTaskModel.cpp
  core/BranchingRepresentation.cpp
  core/Contact.cpp
  core/Dynamics.cpp
  core/Kinematics.cpp
  core/RobotControlModel.cpp
  core/MobileManipulatorServoBehaviors.cpp
  core/BehaviorDescription.cpp
  core/BehaviorFactory.cpp
  core/TaskDescription.cpp
  core/TaskSet.cpp
  core/SAIMatrixAPI.cpp
  core/SAIVectorAPI.cpp

  motion/COMTask.cpp
  motion/FloatBehavior.cpp
  motion/FrictionPosture.cpp
  motion/JointLimitConstraint.cpp
  motion/JointTask.cpp
  motion/ObstacleAvoidanceTask.cpp
  motion/OrientationTask.cpp
  motion/PositionTask.cpp
  motion/PostureBehavior.cpp
  motion/WholeBodyPosture.cpp

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

  parse/BRBuilder.cpp
  parse/BRParser.cpp
  parse/BehaviorParser.cpp
  parse/OsimBRParser.cpp
  parse/TiXmlBRParser.cpp
  parse/taoRepCreator/CTaoRepCreator.cpp

  util/tao_util.cpp
  util/ShuffleVector.cpp
  util/saiTime.cpp
  util/DataLog.cpp
  util/RecorderImpl.cpp
  util/dump.cpp
  util/urdf_to_tao.cpp
  
  bin/builtin.cpp
  bin/attributes.cpp
  bin/DirectoryCmdServer.cpp
  bin/ModelProcess.cpp
  bin/ServoModelProcess.cpp
  bin/ServoProcess.cpp
  bin/TaskModelListener.cpp
  bin/options.cpp
  )

target_link_libraries (Stanford_WBC ${LIBS})

install (TARGETS Stanford_WBC DESTINATION lib)

install (DIRECTORY ./
  DESTINATION include/wbc
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)
