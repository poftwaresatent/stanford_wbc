include_directories (
  ../jspace
  ../tao
  ../3rdparty/reflexxes_otg/include
  ../3rdparty/yaml-cpp/include
  ../3rdparty/gtest-1.6.0/include
  include
  )

add_library (opspace SHARED
  src/pseudo_inverse.cpp
  src/Parameter.cpp
  src/Task.cpp
  src/Factory.cpp
  src/TypeIOTGCursor.cpp
  src/Controller.cpp
  src/ClassicTaskPostureController.cpp
  src/task_library.cpp
  src/skill_library.cpp
  src/parse_yaml.cpp
  src/Skill.cpp
  )
target_link_libraries (opspace jspace reflexxes_otg yaml-cpp)

add_executable (testTask src/testTask.cpp)
target_link_libraries (testTask opspace jspace_test gtest pthread)

add_executable (testFactory src/testFactory.cpp)
target_link_libraries (testFactory opspace gtest pthread)
