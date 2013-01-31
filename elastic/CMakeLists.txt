include_directories (
  ../jspace
#  ../tao
#  ../opspace/include
  )

add_definitions (-DTESTELASTIC_XML_PATH_STR="${CMAKE_CURRENT_SOURCE_DIR}/testelastic.xml")

find_package(GTK2 REQUIRED gtk)
if (GTK2_FOUND)
  message ("*** found GTK2 (required for now)")
  include_directories (${GTK2_INCLUDE_DIRS})
else (GTK2_FOUND)
  message (FATAL_ERROR "Please install GTK2 development files (or tweak build sys to make GTK optional).")
endif (GTK2_FOUND)

list (APPEND SRCS
  elastic.cpp
  )

add_library (elastic SHARED ${SRCS})

add_executable (testelastic testelastic.cpp)
target_link_libraries (testelastic elastic ${GTK2_LIBRARIES})
