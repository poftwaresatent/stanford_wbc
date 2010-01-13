### included via subdir() command in top-level CMakeLists.txt

#add_library (jspace SHARED JSpaceModel.cpp)

add_executable (example example.cpp)
target_link_libraries (example Stanford_WBC)# jspace)
