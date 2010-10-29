add_library (
  wbc_tinyxml SHARED
  wbc_tinyxml/wbc_tinystr.cpp
  wbc_tinyxml/wbc_tinyxml.cpp
  wbc_tinyxml/wbc_tinyxmlerror.cpp
  wbc_tinyxml/wbc_tinyxmlparser.cpp)

target_link_libraries (wbc_tinyxml ${MAYBE_GCOV})

include_directories (.)

add_executable (xmltest tests/xmltest.cpp)
target_link_libraries (xmltest wbc_tinyxml ${MAYBE_GCOV})

install (FILES
  wbc_tinyxml/wbc_tinyxml.h
  wbc_tinyxml/wbc_tinystr.h
  DESTINATION include/wbc_tinyxml)

install (TARGETS
  wbc_tinyxml
  DESTINATION lib)
