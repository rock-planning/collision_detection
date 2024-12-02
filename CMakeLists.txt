cmake_minimum_required(VERSION 3.10)
project(collision_detection)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(TARGET_NAME ${PROJECT_NAME})

# Boost configuration
set(BOOST_ROOT /home/rsc/rsc/projects/libraries/dfki/dependancies/boost_1_82_0)
set(BOOST_LIBRARYDIR ${BOOST_ROOT}/stage/lib)
set(BOOST_INCLUDEDIR ${BOOST_ROOT})

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

# Find required packages using pkg-config
include(FindPkgConfig)
pkg_check_modules(BASE_TYPES REQUIRED base_types)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(SRDFDOM REQUIRED srdfdom)
# pkg_check_modules(CCD REQUIRED ccd)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(FCL REQUIRED fcl)
pkg_check_modules(BASE_LOGGING REQUIRED base_logging)
pkg_check_modules(BULLET REQUIRED bullet)

# # Include Bullet's headers
# include_directories(${BULLET_INCLUDE_DIRS})

# If FCL is found, configure it
if(FCL_FOUND)
    message(STATUS "FCL VERSION ${FCL_VERSION}")
    message(STATUS "FCL INCLUDE DIRS ${FCL_INCLUDE_DIRS}")
    message(STATUS "FCL LIBRARY DIRS ${FCL_LIBRARY_DIRS}")

    # Include FCL headers and set library directories
    include_directories(${FCL_INCLUDE_DIRS})
    link_directories(${FCL_LIBRARY_DIRS})
else()
    message(WARNING "FCL not found")
endif()

# Define the abstract_collision_detection library
add_library(abstract_collision_detection SHARED
    src/abstract/AbstractCollisionDetection.cpp
)

# Include directories for dependencies
set(COLLISION_LIB_INCLUDES 
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${KDL_PARSER_INCLUDE_DIRS}
    ${URDF_INCLUDE_DIRS} 
    ${URDFHEADER_INCLUDE_DIRS} 
    ${SRDFDOM_INCLUDE_DIRS} 
    ${OCTOMAP_INCLUDE_DIRS} 
    ${FCL_INCLUDE_DIRS}
    ${BASE_TYPES_INCLUDE_DIRS} 
    ${BASE_LOGGING_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
)

# Set libraries to link with
set(COLLISION_DEP_LIBS
    Threads::Threads
    Eigen3::Eigen
    ${Boost_SYSTEM_LIBRARY} 
    ${Boost_FILESYSTEM_LIBRARY} 
    ${KDL_PARSER_LIBRARIES} 
    ${URDF_LIBRARIES} 
    ${URDFHEADER_LIBRARIES}
    ${SRDFDOM_LIBRARIES} 
    ${OCTOMAP_LIBRARIES} 
    ${FCL_LIBRARIES}
    ${BASE_TYPES_LIBRARY} 
    ${BASE_LOGGING_LIBRARY}
    ${BULLET_LIBRARIES}
)

target_link_libraries(abstract_collision_detection PUBLIC ${COLLISION_DEP_LIBS}
)

target_include_directories(abstract_collision_detection PUBLIC ${COLLISION_LIB_INCLUDES}
)

# Define the fcl_wrapper library if FCL is found
if(FCL_FOUND)
    add_library(fcl_wrapper SHARED
        src/fcl_wrapper/FCLCollisionDetection.cpp
        src/fcl_wrapper/CollisionObjectAssociatedData.cpp
    )

    target_link_libraries(fcl_wrapper PUBLIC 
        abstract_collision_detection
        ${CCD_LIBRARIES}
        ${COLLISION_DEP_LIBS}
    )

    target_include_directories(fcl_wrapper PUBLIC 
        ${CCD_INCLUDE_DIRS}
        ${COLLISION_LIB_INCLUDES}
    )
endif()

# Define the final collision_detection library
add_library(collision_detection SHARED
    src/CollisionFactory.cpp
)

target_link_libraries(collision_detection
    fcl_wrapper
)

# Link to Bullet libraries
link_directories(${BULLET_LIBRARY_DIRS})
link_directories(${BOOST_LIBRARYDIR})
# target_link_libraries(collision_detection ${BULLET_LIBRARIES})

# target_include_directories(collision_detection PUBLIC ${COLLISION_LIB_INCLUDES})

# Set package information
set(TARGET_NAME ${PROJECT_NAME})
set(PROJECT_DESCRIPTION "Collection of collision detection libraries")

# # Create the pkg-config .pc file
# configure_file(${CMAKE_SOURCE_DIR}/collision_detection.pc.in ${CMAKE_BINARY_DIR}/collision_detection.pc @ONLY)

# Package configuration
list(APPEND PKGCONFIG_REQUIRES kdl_parser urdfdom urdfdom_headers srdfdom octomap fcl base_types base_logging)
list(JOIN PKGCONFIG_REQUIRES " " PKGCONFIG_REQUIRES_STR)
configure_file("${CMAKE_SOURCE_DIR}/${PROJECT_NAME}.pc.in" "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc" @ONLY)

# Install headers and libraries
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)