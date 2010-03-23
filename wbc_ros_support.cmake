cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)

# Try to find ROS. If found, try to find URDF. If everything works as
# desired, it ends up setting HAVE_ROS=true and sets up the include
# and link directives for inclusion of the URDF package.
#
# You can set TRY_ROS=false on the CMake command line in order to skip
# all of this.
#
macro (wbc_find_urdf)
  
  if (NOT TRY_ROS)
    set (TRY_ROS true)
  endif (NOT TRY_ROS)
  
  if (TRY_ROS)
    if (NOT $ENV{ROS_ROOT} STREQUAL "")
      if (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
	message ("[WBC] WARNING ROS_ROOT is set but rosbuild.cmake not found")
	set (HAVE_ROS-NOTFOUND)
      else (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
	set (HAVE_ROS true)
	include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
	rosbuild_find_ros_package (urdf)
	if (${urdf_PACKAGE_PATH} STREQUAL "")
	  message (FATAL_ERROR "ROS support enabled but urdf package not found, try running 'rosmake urdf'")
	else (${urdf_PACKAGE_PATH} STREQUAL "")
	  message ("[WBC] enabling ROS support for URDF")
	  rosbuild_invoke_rospack (urdf wbc_ros_support temp cflags-only-I)
	  include_directories (${wbc_ros_support_temp})
	  rosbuild_invoke_rospack (urdf wbc_ros_support temp cflags-only-other)
	  add_definitions (${wbc_ros_support_temp})
	  rosbuild_invoke_rospack (urdf wbc_ros_support temp libs-only-L)
	  link_directories (${wbc_ros_support_temp})
	  rosbuild_invoke_rospack (urdf wbc_ros_support temp libs-only-l)
	  list (APPEND LIBS ${wbc_ros_support_temp})
	endif (${urdf_PACKAGE_PATH} STREQUAL "")
      endif (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
    endif (NOT $ENV{ROS_ROOT} STREQUAL "")
    if (NOT HAVE_ROS)
      message ("ROS support disabled, will skip URDF conversion stuff")
    endif (NOT HAVE_ROS)
  endif (TRY_ROS)
  
endmacro (wbc_find_urdf)
