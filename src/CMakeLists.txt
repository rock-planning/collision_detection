project(collision_detection)


if(POLICY CMP0015)
  cmake_policy(SET CMP0015 NEW)
endif()


INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
pkg_check_modules(BASE_TYPES REQUIRED base-types)
pkg_search_module(BASE_LOGGING REQUIRED base-logging)
pkg_check_modules(ASSIMP REQUIRED assimp)
pkg_check_modules(SRDF REQUIRED srdfdom)
pkg_check_modules(OCTOMAP REQUIRED octomap)

set(COLLISION_DETECTION_INCLUDES ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${BASE_TYPES_INCLUDE_DIRS} 
                                 ${BASE_LOGGING_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${SRDF_INCLUDE_DIRS} 
                                 ${OCTOMAP_INCLUDE_DIRS})

set(COLLISION_DETECTION_LINK_DIRS ${BASE_TYPES_LIBRARY_DIRS} ${BASE_LOGGING_LIBRARY_DIRS} 
                                  ${ASSIMP_LIBRARY_DIRS} ${SRDF_LIBRARY_DIRS}
                                  ${OCTOMAP_LIBRARY_DIRS})

set(COLLISION_DETECTION_DEP_LIBS ${BASE_TYPES_LIBRARIES} ${BASE_LOGGING_LIBRARIES} ${ASSIMP_LIBRARIES}
                                 ${SRDF_LIBRARIES} ${OCTOMAP_LIBRARIES})

include_directories(${COLLISION_DETECTION_INCLUDES})

link_directories(${COLLISION_DETECTION_LINK_DIRS})

add_library(abstract_collision_detection SHARED
            ${CMAKE_CURRENT_LIST_DIR}/abstract/AbstractCollisionDetection.cpp
)
target_link_libraries(abstract_collision_detection  ${COLLISION_DETECTION_DEP_LIBS})


# FCL Wrapper
pkg_check_modules(FCL REQUIRED fcl)
#pkg_check_modules(FCL fcl QUIET)
#find_package(fcl QUIET)

if( FCL_FOUND)

    message(STATUS "FCL VERSION ${FCL_VERSION}")
    include_directories(${FCL_INCLUDE_DIRS} ${COLLISION_DETECTION_INCLUDES} )

    add_library(fcl_wrapper SHARED
                ${CMAKE_CURRENT_LIST_DIR}/fcl_wrapper/FCLCollisionDetection.cpp
                ${CMAKE_CURRENT_LIST_DIR}/fcl_wrapper/CollisionObjectAssociatedData.cpp
    )

    target_link_libraries(fcl_wrapper ${FCL_LIBRARIES} ${COLLISION_DETECTION_DEP_LIBS} abstract_collision_detection)
    # we add fcl to DEPS_PKGCONFIG
    list(APPEND PKGCONFIG_REQUIRES fcl)
else()
    message("NO FCL FOUND")

endif()

add_library(collision_detection SHARED
            ${CMAKE_CURRENT_LIST_DIR}/CollisionFactory.cpp
)
target_link_libraries(collision_detection  abstract_collision_detection fcl_wrapper)

set(TARGET_NAME ${PROJECT_NAME})
set(PROJECT_DESCRIPTION "Collection of collision detection libraries")
#set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS};${COLLISION_DETECTION_INCLUDES}")
set(PKGCONFIG_LIBS "${PKGCONFIG_LIBS} -labstract_collision_detection -lfcl_wrapper")
# now we add all the required libraries in PKGCONFIG_REQUIRES macro
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES base-logging)
list(APPEND PKGCONFIG_REQUIRES srdfdom)
list(APPEND PKGCONFIG_REQUIRES assimp)
list(APPEND PKGCONFIG_REQUIRES octomap)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/collision_detection DESTINATION include)
install(TARGETS abstract_collision_detection DESTINATION lib)
install(TARGETS fcl_wrapper DESTINATION lib)
install(TARGETS collision_detection DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 





