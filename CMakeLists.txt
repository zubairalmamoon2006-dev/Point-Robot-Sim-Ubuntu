cmake_minimum_required(VERSION 3.15)
project(PointRobotSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared library: everything reusable across phases
add_library(robot_core
    src/Robot.cpp
    src/PController.cpp
)
target_include_directories(robot_core PUBLIC include)

# Phase 0: integrator validation
add_executable(phase0_test src/main_phase0.cpp)
target_link_libraries(phase0_test PRIVATE robot_core)

# Phase 1: go-to-target controller
add_executable(phase1_test src/main_phase1.cpp)
target_link_libraries(phase1_test PRIVATE robot_core)
