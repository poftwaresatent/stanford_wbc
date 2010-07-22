cmake_minimum_required (VERSION 2.6)

include (../../../wbc.cmake)

### this plugin only makes sense if we have libnetwrapper... which is
### searched for in the wbc.cmake module.

if (HAVE_NETWRAP)
  wbc_add_plugin (nwrobot plugin.cpp RobotNetWrapper.cpp)
  target_link_libraries (nwrobot netwrapper ${MAYBE_GCOV})
else (HAVE_NETWRAP)
  message ("WARNING libnetwrapper not found, cannot build the nwrobot plugin")
endif (HAVE_NETWRAP)
