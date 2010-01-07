### included via subdir() command in top-level CMakeLists.txt

list (APPEND LIBS
  wbc_tinyxml
  SAI_Matrix
  TAO_Dynamics_Engine
  wbcnet
  dl
  expat
  )

list (APPEND SRCS

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
  util/File.cpp
  util/utc.cpp
  
  bin/builtin.cpp
  bin/attributes.cpp
  bin/DirectoryCmdServer.cpp
  bin/ModelProcess.cpp
  bin/ServoModelProcess.cpp
  bin/ServoProcess.cpp
  bin/TaskModelListener.cpp
  bin/options.cpp
  bin/Process.cpp
  bin/directory.cpp
  bin/UserProcess.cpp
  )

##################################################
### detect optional xmlrpc++ stuff

include (CheckIncludeFileCXX)

check_include_file_cxx (XmlRpc.h WBC_HAVE_XMLRPC_HEADER)
if (${WBC_HAVE_XMLRPC_HEADER})
  message ("Found XmlRpc++ headers")
  if (XMLRPC_DIR)
    find_library (WBC_HAVE_XMLRPC_LIB XmlRpc PATHS ${XMLRPC_DIR} ${XMLRPC_DIR}/lib)
  else (XMLRPC_DIR)
    find_library (WBC_HAVE_XMLRPC_LIB XmlRpc)
  endif (XMLRPC_DIR)
  if (WBC_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
    message (FATAL_ERROR "XmlRpc++ library not found, although the header XmlRpc.h was found")
  else (WBC_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
    add_definitions (-DHAVE_XMLRPC)
    list (APPEND SRCS bin/XMLRPCDirectoryServer.cpp)
    list (APPEND LIBS XmlRpc)
  endif (WBC_HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
else (${WBC_HAVE_XMLRPC_HEADER})
  message ("WARNING XmlRpc++ not found, or the XmlRpc.h header failed to compile")
  message ("  You can install it from the stanford-wbc 3rdparty directory")
  message ("  or from its origin on http://xmlrpcpp.sourceforge.net/ of course.")
  message ("  After installation, use the XMLRPC_DIR variable to point to the correct location.")
  message ("  You can either set it in the environment or using -DXMLRPC_DIR:path=/where/ever")
  message ("  on the cmake command line.")
endif (${WBC_HAVE_XMLRPC_HEADER})

### done with optional xmlrpc++ stuff
##################################################

add_library (Stanford_WBC SHARED ${SRCS})

target_link_libraries (Stanford_WBC ${LIBS})

install (TARGETS Stanford_WBC DESTINATION lib)

install (DIRECTORY ./
  DESTINATION include/wbc
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)
