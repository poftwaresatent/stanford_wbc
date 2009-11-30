cmake_minimum_required (VERSION 2.6)

# Well... this entire block serves to pull-in the wbc.cmake macro
# definitions. But we want to support separate installations as well
# as devkit-style setups. So we have to look in two places, depending
# on circumstances.
if (NOT WBC_ROOT)
  set (WBC_ROOT $ENV{WBC_ROOT})
endif (NOT WBC_ROOT)
if (WBC_ROOT)
  if (NOT EXISTS ${WBC_ROOT}/share/wbc.cmake)
    message ("[WBC] hmm... looks like a devkit-style build setup...")
    if (NOT Stanford_WBC_SOURCE_DIR)
      message (FATAL_ERROR "The Stanford_WBC_SOURCE_DIR variable is not set, I don't know where to look for the wbc.cmake file!.")
    endif (NOT Stanford_WBC_SOURCE_DIR)
    if (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
      include (${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
    else (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
      message (FATAL_ERROR "Neither ${WBC_ROOT}/share/wbc.cmake nor ${Stanford_WBC_SOURCE_DIR}/wbc.cmake exist, giving up.")
    endif (EXISTS ${Stanford_WBC_SOURCE_DIR}/wbc.cmake)
  else (NOT EXISTS ${WBC_ROOT}/share/wbc.cmake)
    include (${WBC_ROOT}/share/wbc.cmake)
  endif (NOT EXISTS ${WBC_ROOT}/share/wbc.cmake)
else (WBC_ROOT)
  message (FATAL_ERROR "WBC_ROOT must be set either in the environment or as a CMake variable. Follow the instructions in the README and have a look at the installation instructions on http://stanford-wbc.sourceforge.net/")
endif (WBC_ROOT)

### this plugin only makes sense if we have libnetwrapper...
# try to get NETWRAP_DIR from CMake or environment
# 1. CMake variable NETWRAP_DIR
# 2. environment variable NETWRAP_DIR
if (NOT NETWRAP_DIR)
  set (NETWRAP_DIR $ENV{NETWRAP_DIR})
  if (NETWRAP_DIR)
    message ("using NETWRAP_DIR from environment: ${NETWRAP_DIR}")
  endif (NETWRAP_DIR)
else (NOT NETWRAP_DIR)
  message ("using NETWRAP_DIR from CMake: ${NETWRAP_DIR}")
endif (NOT NETWRAP_DIR)
if (NETWRAP_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES ${NETWRAP_DIR}/include)
  include_directories (${NETWRAP_DIR}/include)
  link_directories (${NETWRAP_DIR}/lib)
  check_include_file_cxx (NetWrapper.h HAVE_NETWRAP)
  if (HAVE_NETWRAP)
    message ("found NetWrapper.h")
    add_definitions (-DHAVE_NETWRAP)
    wbc_add_plugin (nwrobot plugin.cpp RobotNetWrapper.cpp)
    target_link_libraries (nwrobot netwrapper)
  else (HAVE_NETWRAP)
    message (FATAL_ERROR "NetWrapper.h not found even though NETWRAP_DIR was set to ${NETWRAP_DIR}")
  endif (HAVE_NETWRAP)
endif (NETWRAP_DIR)

###############################################################################
### the wbc_add_plugin() command appears inside the check for libnetwrapper ###
###############################################################################
