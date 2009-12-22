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

wbc_add_plugin (fake plugin.cpp RobotFake.cpp DebugBehavior.cpp RawController.cpp)
