cmake_minimum_required (VERSION 2.4)
if (COMMAND CMAKE_POLICY)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
endif (COMMAND CMAKE_POLICY)

include (../wbc.cmake)
wbc_init (wbc_tinyxml)

add_library (
  wbc_tinyxml SHARED
  wbc_tinyxml/wbc_tinystr.cpp
  wbc_tinyxml/wbc_tinyxml.cpp
  wbc_tinyxml/wbc_tinyxmlerror.cpp
  wbc_tinyxml/wbc_tinyxmlparser.cpp)

include_directories (${wbc_tinyxml_SOURCE_DIR})

install (FILES
  wbc_tinyxml/wbc_tinyxml.h
  wbc_tinyxml/wbc_tinystr.h
  DESTINATION include/wbc_tinyxml)

install (TARGETS
  wbc_tinyxml
  DESTINATION lib)
