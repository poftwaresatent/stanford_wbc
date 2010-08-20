cmake_minimum_required (VERSION 2.6)

include (wbc.cmake)
wbc_init (Stanford_WBC_Umbrella)

list (APPEND CMAKE_REQUIRED_INCLUDES ${Stanford_WBC_Umbrella_SOURCE_DIR})
include_directories (${Stanford_WBC_Umbrella_SOURCE_DIR} ${Stanford_WBC_Umbrella_SOURCE_DIR}/wbcnet)


##################################################
### A note about log4cxx: if wbcnet finds it and we don't, we'll get
### screwed. If we link with things that come from ROS and we don't
### find log4cxx, we'll likewise confuse the build. Gawd.


##################################################
# Users can request a custom hardcoded start of WBC_PLUGIN_PATH, but
# we fall back on some "reasonable default" ... this should probably
# not appear in the top-level CMakeLists.txt anyhow.

if (NOT WBC_PLUGIN_PATH)
  message ("setting WBC_PLUGIN_PATH to ${Stanford_WBC_Umbrella_BINARY_DIR}/plugins")
  set (WBC_PLUGIN_PATH ${Stanford_WBC_Umbrella_BINARY_DIR}/plugins)
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
  jspace
  wbc
  )

if (APPLE)
  message ("[WBC] Skipping plugins on Mac OS X")
else (APPLE)
  subdirs (wbc_plugins)
endif (APPLE)

install (FILES wbc.cmake DESTINATION share)

