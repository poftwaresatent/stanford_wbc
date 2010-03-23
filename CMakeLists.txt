cmake_minimum_required (VERSION 2.6)

include (macros.cmake)
wbc_init (Stanford_WBC_Umbrella)

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
  saimatrix
  tao
  wbc_tinyxml
  wbcnet
##  robarch
#  jspace
#  wbc
#  wbc/plugins/fake
#  wbc/plugins/robotlog
#  wbc/plugins/nwrobot
#  wbc/plugins/netrob
#  wbc/plugins/rosrob
#  tests
#  applications
  )
