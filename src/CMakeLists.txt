project(collision_detection)


if(POLICY CMP0015)
  cmake_policy(SET CMP0015 NEW)
endif()


INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
#find_package(orocos_kdl REQUIRED)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)
pkg_check_modules(SRDF REQUIRED srdfdom)


pkg_check_modules(ASSIMP REQUIRED assimp)
message(STATUS "ASSIMP VERSION ${ASSIMP_VERSION}")

set(COLLISION_DETECTION_INCLUDES ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS}  
				 ${ASSIMP_INCLUDE_DIRS} ${SRDF_INCLUDE_DIRS} )

set(COLLISION_DETECTION_LINK_DIRS ${ASSIMP_LIBRARY_DIRS} ${SRDF_LIBRARY_DIRS}) 
include_directories(${COLLISION_DETECTION_INCLUDES})
link_directories(${COLLISION_DETECTION_LINK_DIRS})

add_library(abstract_collision_detection SHARED
            ${CMAKE_CURRENT_LIST_DIR}/abstract/AbstractCollisionDetection.cpp
)
target_link_libraries(abstract_collision_detection  ${ASSIMP_LIBRARIES} ${SRDF_LIBRARIES})


# FCL Wrapper
pkg_check_modules(FCL fcl QUIET)
#find_package(fcl QUIET)

if( FCL_FOUND)

    message(STATUS "FCL VERSION ${FCL_VERSION}")
    include_directories(${FCL_INCLUDE_DIRS} ${COLLISION_DETECTION_INCLUDES} )

    add_library(fcl_wrapper SHARED
                ${CMAKE_CURRENT_LIST_DIR}/fcl_wrapper/FCLCollisionDetection.cpp
                ${CMAKE_CURRENT_LIST_DIR}/fcl_wrapper/CollisionObjectAssociatedData.cpp
    )

    target_link_libraries(fcl_wrapper ${FCL_LIBRARIES} abstract_collision_detection)
else()
    message("NO FCL FOUND")

endif()

add_library(collision_detection SHARED
            ${CMAKE_CURRENT_LIST_DIR}/CollisionFactory.cpp
)
target_link_libraries(collision_detection  abstract_collision_detection fcl_wrapper)

set(TARGET_NAME ${PROJECT_NAME})
#set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS};${COLLISION_DETECTION_INCLUDES}")
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include/collision_detection)
install(TARGETS abstract_collision_detection DESTINATION lib)
install(TARGETS fcl_wrapper DESTINATION lib)
install(TARGETS collision_detection DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 





