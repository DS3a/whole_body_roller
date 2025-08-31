# Template CMakeLists.txt for a controller package
cmake_minimum_required(VERSION 3.10)
project(whole_body_roller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Pinocchio using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# add eigen as a dependency
find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)

# Include directories and link libraries
include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})
add_definitions(${PINOCCHIO_CFLAGS_OTHER})

include_directories(include)

# add_library(quadprogpp STATIC
#     QuadProgpp/src/Array.cc
#     QuadProgpp/src/QuadProg++.cc)
# target_include_directories(quadprogpp PUBLIC
#     QuadProgpp/src)


# Source files
# file(GLOB SRC_FILES src/*.cpp)

add_library(whole_body_roller
    src/control_decision_variables.cpp
    src/constraint.cpp
    src/dynamics.cpp
    src/roller.cpp
)

target_link_libraries(whole_body_roller
    # quadprogpp
    casadi
    ${PINOCCHIO_LIBRARIES}
)
# Dependencies
set(DEPENDENCIES
    pthread
    glfw
    yaml-cpp
)

# Executable target
add_executable(whole_body_roller_test src/test.cpp)
target_link_libraries(whole_body_roller_test 
    whole_body_roller 
    casadi)
target_include_directories(whole_body_roller PUBLIC
    include
    ${DEPENDENCIES}
    ${PINOCCHIO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

