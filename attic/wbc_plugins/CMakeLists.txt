cmake_minimum_required (VERSION 2.6)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (Stanford_WBC_Plugins)

include_directories (
  ../wbc
  ../wbcnet
  ../jspace
  ../saimatrix
  ../tao
  )

subdirs (
  wbc_plugins/fake
  wbc_plugins/robotlog
  wbc_plugins/nwrobot
  wbc_plugins/netrob
  )

file (GLOB headers "fake/*.hpp")
install (FILES ${headers} DESTINATION include/wbc_plugins/fake)

file (GLOB headers "robotlog/*.hpp")
install (FILES ${headers} DESTINATION include/wbc_plugins/robotlog)

file (GLOB headers "nwrobot/*.hpp")
install (FILES ${headers} DESTINATION include/wbc_plugins/nwrobot)

file (GLOB headers "netrob/*.hpp")
install (FILES ${headers} DESTINATION include/wbc_plugins/netrob)
