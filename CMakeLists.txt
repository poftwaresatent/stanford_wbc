cmake_minimum_required (VERSION 2.6)

include (wbc.cmake)
wbc_init (Stanford_WBC)

list (APPEND CMAKE_REQUIRED_INCLUDES ${Stanford_WBC_SOURCE_DIR})
include_directories (${Stanford_WBC_SOURCE_DIR} ${Stanford_WBC_SOURCE_DIR}/wbcnet)


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


##################################################
### A note about log4cxx: if wbcnet finds it and we don't, we'll get
### screwed. If we link with things that come from ROS and we don't
### find log4cxx, we'll likewise confuse the build. Gawd.


##################################################
# users can request a custom hardcoded start of WBC_PLUGIN_PATH, but we fall back on 

if (NOT WBC_PLUGIN_PATH)
  message ("setting WBC_PLUGIN_PATH to ${Stanford_WBC_BINARY_DIR}/plugins")
  set (WBC_PLUGIN_PATH ${Stanford_WBC_BINARY_DIR}/plugins)
else (NOT WBC_PLUGIN_PATH)
  message ("WBC_PLUGIN_PATH already set to ${WBC_PLUGIN_PATH}")
endif (NOT WBC_PLUGIN_PATH)
add_definitions (-DWBC_PLUGIN_PATH_STR="${WBC_PLUGIN_PATH}")


##################################################
# If we have ROS, then we might even be able to support URDF-to-TAO
# conversion. And we have to put this detection here at the top
# because users of the Stanford_WBC library do not want to have to
# worry about linking (or not) with the URDF lib.

if (NOT TRY_ROS)
  set (TRY_ROS true)
endif (NOT TRY_ROS)
if (TRY_ROS)
  if (NOT $ENV{ROS_ROOT} STREQUAL "")
    if (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      message ("WARNING ROS_ROOT is set but rosbuild.cmake not found")
      set (HAVE_ROS-NOTFOUND)
    else (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      set (HAVE_ROS true)
      include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      rosbuild_find_ros_package (urdf)
      if (${urdf_PACKAGE_PATH} STREQUAL "")
	message ("WARNING urdf ROS package not found")
      else (${urdf_PACKAGE_PATH} STREQUAL "")
	rosbuild_invoke_rospack (urdf playtao temp cflags-only-I)
	include_directories (${playtao_temp})
	rosbuild_invoke_rospack (urdf playtao temp cflags-only-other)
	add_definitions (${playtao_temp})
	rosbuild_invoke_rospack (urdf playtao temp libs-only-L)
	link_directories (${playtao_temp})
	rosbuild_invoke_rospack (urdf playtao temp libs-only-l)
	list (APPEND LIBS ${playtao_temp})
	add_definitions (-DHAVE_URDF)
      endif (${urdf_PACKAGE_PATH} STREQUAL "")
    endif (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  endif (NOT $ENV{ROS_ROOT} STREQUAL "")
  if (NOT HAVE_ROS)
    message ("ROS support disabled, will skip URDF conversion stuff")
  endif (NOT HAVE_ROS)
endif (TRY_ROS)


##################################################
### This shouldn'b be necessary, I think CMake just is not very
### supportive of linking with external libraries.

if (NOT NETWRAP_DIR)
  set (NETWRAP_DIR $ENV{NETWRAP_DIR})
endif (NOT NETWRAP_DIR)
if (NETWRAP_DIR)
  link_directories (${NETWRAP_DIR}/lib)
endif (NETWRAP_DIR)


##################################################
### go into subdirs

subdirs (
  jspace
  wbc_tinyxml
  saimatrix
  tao
  wbcnet
  wbc
  plugins/fake
  plugins/robotlog
  plugins/nwrobot
  plugins/netrob
  plugins/rosrob
  tests
  applications
  )

install (FILES wbc.cmake DESTINATION share)
