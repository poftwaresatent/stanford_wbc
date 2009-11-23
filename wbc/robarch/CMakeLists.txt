list (APPEND ROBARCH_SRC
  glob_rob_ds/SGlobalRobotDS.hpp
  glob_rob_ds/SControllerRobotLink.hpp
  glob_rob_ds/SArachiRobotLink.hpp
  glob_rob_ds/SGraphicsRobotLink.hpp
  CRobotArchitect.cpp
  )

list (APPEND ROBARCH_SRC
			osimarchitect/parser/SkeletonCoordinate.cpp
	osimarchitect/parser/CTransformAxisObj.cpp
	osimarchitect/parser/SkeletonMarker.cpp
	osimarchitect/parser/CWrapCylinderNew.cpp
	osimarchitect/CCoordObj.cpp
	osimarchitect/parser/CJointNew.cpp
	osimarchitect/parser/CSkeletonLinkNew.cpp	
	osimarchitect/parser/CWrapEllipsoidNew.cpp	
	osimarchitect/parser/CSkeletonModelNew.cpp
	osimarchitect/parser/CMuscleDefinition.cpp
	osimarchitect/COsimArchitect.cpp)
	
include_directories(
								     osimarchitect)

add_library (robarch SHARED ${ROBARCH_SRC})

include_directories (.
										 ../tao
                     ../tao/matrix
								     ../tao/dynamics
								     ../tao/utility
								     ../saimatrix
								     )
add_dependencies (robarch symlink-wbc)
