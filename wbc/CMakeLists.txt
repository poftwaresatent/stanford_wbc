cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (Stanford_WBC)
wbc_find_urdf ()

if (HAVE_CURSES)
  list (APPEND LIBS curses)
else (HAVE_CURSES)
  message ("WARNING curses not found")
  message ("        keypress-commands will not work")
  message ("        you should seriously consider installing the package")
  message ("        (e.g. libncurses5-dev or so)")
endif (HAVE_CURSES)

if (NOT HAVE_EXPAT)
  message (FATAL_ERROR "Expat not found. Please install it (e.g. `sudo apt-get install libexpat1-dev')")
endif (NOT HAVE_EXPAT)

list (APPEND LIBS
  wbc_tinyxml
  SAI_Matrix
  TAO_Dynamics_Engine
  wbcnet
  jspace
  robarch
  dl
  expat
  )

list (APPEND SRCS

  wbc/core/Plugin.cpp
  wbc/core/RobotFactory.cpp
  wbc/core/TaskModelBase.cpp
  wbc/core/MobileManipulatorTaskModel.cpp
  wbc/core/BranchingRepresentation.cpp
  wbc/core/Contact.cpp
  wbc/core/Dynamics.cpp
  wbc/core/Kinematics.cpp
  wbc/core/RobotControlModel.cpp
  wbc/core/MobileManipulatorServoBehaviors.cpp
  wbc/core/BehaviorDescription.cpp
  wbc/core/BehaviorFactory.cpp
  wbc/core/TaskDescription.cpp
  wbc/core/TaskSet.cpp
  wbc/core/SAIMatrixAPI.cpp
  wbc/core/SAIVectorAPI.cpp

  wbc/motion/COMTask.cpp
  wbc/motion/FloatBehavior.cpp
  wbc/motion/FrictionPosture.cpp
  wbc/motion/JointLimitConstraint.cpp
  wbc/motion/JointTask.cpp
  wbc/motion/ObstacleAvoidanceTask.cpp
  wbc/motion/OrientationTask.cpp
  wbc/motion/PositionTask.cpp
  wbc/motion/PostureBehavior.cpp
  wbc/motion/WholeBodyPosture.cpp

  wbc/parse/BRParser.cpp
  wbc/parse/BehaviorParser.cpp
  wbc/parse/OsimBRParser.cpp
  wbc/parse/TiXmlBRParser.cpp

  wbc/util/ShuffleVector.cpp
  wbc/util/saiTime.cpp
  wbc/util/DataLog.cpp
  wbc/util/RecorderImpl.cpp
  wbc/util/File.cpp
  wbc/util/utc.cpp
  wbc/util/dtor_check.cpp

  wbc/bin/builtin.cpp
  wbc/bin/attributes.cpp
  wbc/bin/DirectoryCmdServer.cpp
  wbc/bin/ModelProcess.cpp
  wbc/bin/ServoModelProcess.cpp
  wbc/bin/ServoProcess.cpp
  wbc/bin/TaskModelListener.cpp
  wbc/bin/options.cpp
  wbc/bin/Process.cpp
  wbc/bin/directory.cpp
  wbc/bin/UserProcess.cpp
  )

if (HAVE_XMLRPC)
  list (APPEND SRCS wbc/bin/XMLRPCDirectoryServer.cpp)
  list (APPEND LIBS XmlRpc)
endif (HAVE_XMLRPC)

if (HAVE_ROS)
  list (APPEND SRCS
    wbc/ros/Model.cpp
    wbc/ros/BehaviorLibrary.cpp
    )
endif (HAVE_ROS)


add_library (Stanford_WBC SHARED ${SRCS})

include_directories (
  .
  ../saimatrix
  ../tao
  ../wbc_tinyxml
  ../wbcnet
  ../robarch
  ../jspace
  )

target_link_libraries (Stanford_WBC ${LIBS} ${MAYBE_GCOV})

subdirs (
  tests
  applications
  )

install (TARGETS Stanford_WBC DESTINATION lib)

install (DIRECTORY wbc/
  DESTINATION include/wbc
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)
