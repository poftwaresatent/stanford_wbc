cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)



##################################################
#
# wbc_add_plugin (name source1 source2 ... sourceN)
#
# Add a plugin to the whole-body controller. This is basically just a
# wrapper around CMake's standard add_library() command. The plugin
# file will be named according to the CMake rules concerning the given
# name, and the sources passed to the wbc_add_plugin() command will be
# compiled and linked into the plugin.
#
# The plugin can be stored in various locations, depending on your
# setup:
# - if the CMake variable WBC_PLUGIN_PATH is set, the plugin's .so
#   file gets written there
# - otherwise, if WBC_ROOT is set in CMake or the environment, it gets
#   written to ${WBC_ROOT}/plugins
# - otherwise, it ends up in the CMake default location for libraries
macro (wbc_add_plugin PLUGIN_NAME)
  message ("[WBC] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
  message ("[WBC] BEGIN configuring plugin ${PLUGIN_NAME}")
  
  wbc_init (${PLUGIN_NAME})
  
  add_library (${PLUGIN_NAME} MODULE ${ARGN})
  
  set (PLUGIN_OUT_DIR ${WBC_PLUGIN_PATH})
  if (NOT PLUGIN_OUT_DIR)
    # Note that WBC_ROOT can come from the environment, this is done
    # in the wbc_init() macro which we call a couple of lines above.
    if (WBC_ROOT)
      set (PLUGIN_OUT_DIR ${WBC_ROOT}/plugins)
    endif (WBC_ROOT)
  endif (NOT PLUGIN_OUT_DIR)
  
  if (PLUGIN_OUT_DIR)
    message ("[WBC] setting plugin output path to ${PLUGIN_OUT_DIR}")
    set_target_properties (${PLUGIN_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PLUGIN_OUT_DIR})
  else (PLUGIN_OUT_DIR)
    message ("[WBC] falling back on the CMake default plugin output path")
  endif (PLUGIN_OUT_DIR)
  
  install (TARGETS ${PLUGIN_NAME} DESTINATION plugins)
  
  message ("[WBC] FINISHED configuring WBC plugin ${PLUGIN_NAME}")
  message ("[WBC] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
endmacro (wbc_add_plugin)
