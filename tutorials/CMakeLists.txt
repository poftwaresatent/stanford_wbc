include_directories (
  ../jspace
  ../tao
  ../opspace/include
  )

set (FLTK_SKIP_OPENGL ON)
set (FLTK_SKIP_FORMS ON)
set (FLTK_SKIP_IMAGES ON)
set (FLTK_SKIP_FLUID ON)
include (FindFLTK)

add_definitions (-DTUTROB_XML_PATH_STR="${CMAKE_CURRENT_SOURCE_DIR}/tutrob.xml")

add_library (tut STATIC tutsim.cpp)

add_executable (tut00_test tut00_test.cpp)
target_link_libraries (tut00_test tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut01_joint_coupling tut01_joint_coupling.cpp)
target_link_libraries (tut01_joint_coupling tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut02_jtask tut02_jtask.cpp)
target_link_libraries (tut02_jtask tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut03_gravity_compensation tut03_gravity_compensation.cpp)
target_link_libraries (tut03_gravity_compensation tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut04_inertia_coriolis tut04_inertia_coriolis.cpp)
target_link_libraries (tut04_inertia_coriolis tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut05_opspace_and_parameters tut05_opspace_and_parameters.cpp)
target_link_libraries (tut05_opspace_and_parameters tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut06_eepos tut06_eepos.cpp)
target_link_libraries (tut06_eepos tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})
