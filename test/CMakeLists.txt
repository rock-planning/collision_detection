INCLUDE(FindPkgConfig)
find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
find_package(CCD REQUIRED)
pkg_check_modules(BASE_TYPES REQUIRED base-types)
pkg_check_modules(SRDF REQUIRED srdfdom)
pkg_check_modules(OCTOMAP REQUIRED octomap)

include_directories(${PROJECT_SOURCE_DIR}/include ${BASE_TYPES_INCLUDE_DIRS} )
link_directories(${PROJECT_SOURCE_DIR}/build/src/ ${OCTOMAP_LIBRARY_DIRS})
set (EXTERNAL_LIBS octomap ${BASE_TYPES_LIBRARIES} ${CCD_LIBRARIES} ${OCTOMAP_LIBRARIES} ${SRDF_LIBRARIES} fcl  fcl_wrapper collision_detection abstract_collision_detection 
                    ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_collision_detection test_collision_detection.cpp)
target_link_libraries(test_collision_detection ${EXTERNAL_LIBS})

 