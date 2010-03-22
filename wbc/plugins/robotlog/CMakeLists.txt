cmake_minimum_required (VERSION 2.6)

include (../../../wbc.cmake)

### # Well... this entire block serves to pull-in the wbc.cmake macro
### # definitions. But we want to support separate installations as well
### # as devkit-style setups. So we have to look in two places, depending
### # on circumstances.
### if (Stanford_WBC_SOURCE_DIR)
###   message ("Stanford_WBC_SOURCE_DIR: ${Stanford_WBC_SOURCE_DIR}")
###   if (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
###     include (${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
###   else (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
###     message (FATAL_ERROR "There is no ${Stanford_WBC_SOURCE_DIR}/wbc.cmake file.")
###   endif (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
### else (Stanford_WBC_SOURCE_DIR)
###   if (NOT WBC_ROOT)
###     set (WBC_ROOT $ENV{WBC_ROOT})
###   endif (NOT WBC_ROOT)
###   if (NOT WBC_ROOT)
###     message (FATAL_ERROR "Neither Stanford_WBC_SOURCE_DIR nor WBC_ROOT is set, where is the wbc.cmake file?")
###   endif (NOT WBC_ROOT)
###   message ("WBC_ROOT: ${WBC_ROOT}")
###   if (EXISTS ${WBC_ROOT}/share/wbc.cmake)
###     include (${WBC_ROOT}/share/wbc.cmake)
###   else (EXISTS ${WBC_ROOT}/share/wbc.cmake)
###     message (FATAL_ERROR "There is no ${WBC_ROOT}/share/wbc.cmake file.")
###   endif (EXISTS ${WBC_ROOT}/share/wbc.cmake)
### endif (Stanford_WBC_SOURCE_DIR)

list (APPEND LIBS expat Stanford_WBC)

wbc_add_plugin (robotlog plugin.cpp readlog.cpp writelog.cpp)
target_link_libraries (robotlog ${LIBS})

if (HAVE_GTEST)
  add_executable (test test.cpp readlog.cpp writelog.cpp)
  target_link_libraries (test gtest ${LIBS})
endif (HAVE_GTEST)
