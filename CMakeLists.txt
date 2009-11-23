cmake_minimum_required (VERSION 2.6)

include (wbc.cmake)
wbc_init (Stanford_WBC)

list (APPEND CMAKE_REQUIRED_INCLUDES ${Stanford_WBC_SOURCE_DIR})
include_directories (${Stanford_WBC_SOURCE_DIR} ${Stanford_WBC_SOURCE_DIR}/wbcnet)

include (CheckIncludeFileCXX)

check_include_file_cxx (curses.h WBC_HAVE_CURSES)
if (${WBC_HAVE_CURSES})
  message ("Great, found curses.")
  list (APPEND LIBS curses)
else (${WBC_HAVE_CURSES})
  message ("WARNING curses not found")
  message ("        keypress-commands will not work")
  message ("        you should seriously consider installing the package")
  message ("        (e.g. libncurses5-dev or so)")
endif (${WBC_HAVE_CURSES})

check_include_file_cxx (expat.h WBC_HAVE_EXPAT_HEADER)
if (${WBC_HAVE_EXPAT_HEADER})
  message ("Found expat headers.")
  find_library (WBC_HAVE_EXPAT_LIB expat)
  message ("DBG WBC_HAVE_EXPAT_LIB: ${WBC_HAVE_EXPAT_LIB}")
  if (${WBC_HAVE_EXPAT_LIB} STREQUAL "WBC_HAVE_EXPAT_LIB-NOTFOUND")
    message (FATAL_ERROR "Weird: expat headers were found, but not the library. Please install it properly.")
  else (${WBC_HAVE_EXPAT_LIB} STREQUAL "WBC_HAVE_EXPAT_LIB-NOTFOUND")
    message ("Found expat libray.")
  endif (${WBC_HAVE_EXPAT_LIB} STREQUAL "WBC_HAVE_EXPAT_LIB-NOTFOUND")
else (${WBC_HAVE_EXPAT_HEADER})
  message (FATAL_ERROR "Expat not found. Please install it (e.g. `sudo apt-get install libexpat1-dev')")
endif (${WBC_HAVE_EXPAT_HEADER})

find_library (WBC_HAVE_LAPACK lapack)
if (WBC_HAVE_LAPACK)
  message ("found LAPACK")
else (WBC_HAVE_LAPACK)
  message (FATAL_ERROR "LAPACK not found")
endif (WBC_HAVE_LAPACK)


### try to find XmlRpc++, only needed for lib search path... is there a better way to achieve this?

if (NOT XMLRPC_DIR)
  message ("environment XMLRPC_DIR = $ENV{XMLRPC_DIR}")
  set (XMLRPC_DIR $ENV{XMLRPC_DIR})
endif (NOT XMLRPC_DIR)
message ("XMLRPC_DIR is \"${XMLRPC_DIR}\"")
if (XMLRPC_DIR)
  message ("using XMLRPC_DIR")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${XMLRPC_DIR}/src ${XMLRPC_DIR}/include ${XMLRPC_DIR})
  include_directories (${XMLRPC_DIR}/src ${XMLRPC_DIR}/include ${XMLRPC_DIR})
  link_directories (${XMLRPC_DIR}/lib ${XMLRPC_DIR})
endif (XMLRPC_DIR)

### Try to find log4cxx. If wbcnet finds it and we don't, we'll get
### screwed. If we link with things that come from ROS and we don't
### find log4cxx, we'll likewise confuse the build. Gawd.

if (LOG4CXX_DIR)
  message ("Using LOG4CXX_DIR = ${LOG4CXX_DIR}")
  list (APPEND CMAKE_REQUIRED_INCLUDES ${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
  include_directories (${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
  link_directories (${LOG4CXX_DIR}/lib ${LOG4CXX_DIR})
endif (LOG4CXX_DIR)
check_include_file_cxx (log4cxx/logger.h HAVE_LOG4CXX)
if (HAVE_LOG4CXX)
  message ("Found log4cxx.")
  add_definitions (-DHAVE_LOG4CXX)
####  list (APPEND LIBS log4cxx)
endif (HAVE_LOG4CXX)

### hardcode a portion of the plugin search path ###

# try to define WBC_PLUGIN_PATH by trying the following things:
# 1. CMake variable WBC_PLUGIN_PATH
# 2. environment variable WBC_PLUGIN_PATH
# 3. CMake variable WBC_ROOT + "/plugins"
# 4. environment variable WBC_ROOT + "/plugins"
# 5. fallback on CMake variable CMAKE_BINARY_DIR + "/plugins"

if (NOT WBC_PLUGIN_PATH)
  set (WBC_PLUGIN_PATH $ENV{WBC_PLUGIN_PATH})
  if (NOT WBC_PLUGIN_PATH)
    if (NOT WBC_ROOT)
      set (WBC_ROOT $ENV{WBC_ROOT})
    endif (NOT WBC_ROOT)
    if (WBC_ROOT)
      set (WBC_PLUGIN_PATH ${WBC_ROOT}/plugins)
    endif (WBC_ROOT)
  endif (NOT WBC_PLUGIN_PATH)
endif (NOT WBC_PLUGIN_PATH)
if (NOT WBC_PLUGIN_PATH)
  set (WBC_PLUGIN_PATH ${CMAKE_BINARY_DIR}/plugins)
endif (NOT WBC_PLUGIN_PATH)

# The default plugin path gets hardcoded to the beginning of the
# search order in core/Plugin.cpp using the WBC_PLUGIN_PATH_STR
# preprocessor definition

add_definitions (-DWBC_PLUGIN_PATH_STR="${WBC_ROOT}/plugins")


##################################################
# If we have ROS, then we might even be able to support URDF-to-TAO
# conversion. And we have to put this detection here at the top
# because users of the Stanford_WBC library do not want to have to
# worry about linking (or not) with the URDF lib.

if (NOT TRY_ROS)
  set (TRY_ROS true)
endif (NOT TRY_ROS)
if (TRY_ROS)
  message ("DBG ROS_ROOT: $ENV{ROS_ROOT}")
  if (NOT $ENV{ROS_ROOT} STREQUAL "")
    if (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      message ("WARNING ROS_ROOT is set but rosbuild.cmake not found")
      set (HAVE_ROS-NOTFOUND)
    else (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      set (HAVE_ROS true)
      include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
      rosbuild_find_ros_package (urdf)
      message ("DBG urdf_PACKAGE_PATH: ${urdf_PACKAGE_PATH}")
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
    message ("ROS not found, cannot handle URDF")
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


### go into subdirs ###

subdirs (
  wbc_tinyxml
  saimatrix
  tao
  wbcnet
  wbcrun
  wbc
  plugins/fake
  plugins/robotlog
  tests
  applications
  )

install (FILES wbc.cmake DESTINATION share)
