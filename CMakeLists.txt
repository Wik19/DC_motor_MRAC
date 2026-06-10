cmake_minimum_required(VERSION 3.16)
project(dc_motor_mrac CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(sim
    src/main.cpp
    src/motor.cpp
    src/reference_model.cpp
    src/controller.cpp
    src/simulation.cpp
)
target_include_directories(sim PRIVATE include)
