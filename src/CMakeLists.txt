rock_library(collision_detection
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(collision_detection_bin Main.cpp
    DEPS collision_detection)

