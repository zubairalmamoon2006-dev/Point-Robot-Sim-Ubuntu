cmake_minimum_required(VERSION 3.15)
project(PointRobotSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(robot_core
    src/Robot.cpp
    src/PController.cpp
    src/PIDController.cpp
)
target_include_directories(robot_core PUBLIC include)

add_executable(phase0_test src/main_phase0.cpp)
target_link_libraries(phase0_test PRIVATE robot_core)

add_executable(phase1_test src/main_phase1.cpp)
target_link_libraries(phase1_test PRIVATE robot_core)

add_executable(phase2_test src/main_phase2.cpp)
target_link_libraries(phase2_test PRIVATE robot_core)
