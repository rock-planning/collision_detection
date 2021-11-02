project(collision_detection)


if(POLICY CMP0015)
  cmake_policy(SET CMP0015 NEW)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

rock_library(abstract_collision_detection
    SOURCES abstract/AbstractCollisionDetection.cpp
    DEPS_PKGCONFIG
        base-types
        kdl_parser
        srdfdom
        eigen3
        assimp
        ccd
        octomap
)

# FCL Wrapper
pkg_check_modules(FCL REQUIRED fcl)

if( FCL_FOUND)
    message(STATUS "FCL VERSION ${FCL_VERSION}")
    message(STATUS "FCL VERSION ${FCL_INCLUDE_DIRS}")
    message(STATUS "FCL VERSION ${FCL_LIBRARY_DIRS}")
    include_directories(${FCL_INCLUDE_DIRS} ${COLLISION_DETECTION_INCLUDES} )
    link_directories(${FCL_LIBRARY_DIRS})
    rock_library(fcl_wrapper
        SOURCES fcl_wrapper/FCLCollisionDetection.cpp
               fcl_wrapper/CollisionObjectAssociatedData.cpp
        DEPS_PKGCONFIG
            fcl
	    ccd
            base-types
        DEPS
            abstract_collision_detection
    )
    # we add fcl to DEPS_PKGCONFIG
    list(APPEND PKGCONFIG_REQUIRES fcl)
else()
    message("NO FCL FOUND")
endif()

rock_library(collision_detection
    SOURCES CollisionFactory.cpp
    DEPS
        fcl_wrapper
)

set(TARGET_NAME ${PROJECT_NAME})
set(PROJECT_DESCRIPTION "Collection of collision detection libraries")
set(PKGCONFIG_LIBS "${PKGCONFIG_LIBS} -labstract_collision_detection -lfcl_wrapper")
# now we add all the required libraries in PKGCONFIG_REQUIRES macro
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES base-logging)
list(APPEND PKGCONFIG_REQUIRES srdfdom)
list(APPEND PKGCONFIG_REQUIRES assimp)
list(APPEND PKGCONFIG_REQUIRES ccd)
list(APPEND PKGCONFIG_REQUIRES octomap)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/collision_detection DESTINATION include)





