cmake_minimum_required (VERSION 2.6)
include (../../../wbc.cmake)

add_library (netrob SHARED Robot.cpp RawRobot.cpp)
target_link_libraries (netrob Stanford_WBC)

wbc_add_plugin (netrob_plugin plugin.cpp)
target_link_libraries (netrob_plugin netrob)

install (TARGETS netrob DESTINATION lib)

install (DIRECTORY ./
  DESTINATION include/wbc/plugins/netrob
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)
