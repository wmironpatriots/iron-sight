cmake_minimum_required(VERSION 3.15)
project(IRONSIGHT CXX C)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # enable o3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # warn all

option(ENABLE_CLANG_TIDY "Enable clang-tidy for static analysis" ON)

find_package(OpenCV REQUIRED)
find_package(wpilib REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(src)