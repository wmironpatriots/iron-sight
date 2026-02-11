cmake_minimum_required(VERSION 3.15)
project(IRONSIGHT CXX C)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # enable o3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # warn all

option(ENABLE_CLANG_TIDY "Enable clang-tidy for static analysis" ON)

# Add vendor dirs
# add_subdirectory(vendor/json)
# add_subdirectory(vendor/opencv)

# set(OpenCV_DIR "${CMAKE_CURRENT_BINARY_DIR}/vendor/opencv" CACHE PATH "" FORCE)

# add_subdirectory(vendor/apriltag)
# add_subdirectory(vendor/allwpilib)

include(FetchContent)

FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.12.0  # Or a specific hash for stability
)

FetchContent_MakeAvailable(json)

# Fetch apriltag
# FetchContent_Declare(
#   apriltag
#   GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
#   GIT_TAG        v3.4.5
# )

# FetchContent_MakeAvailable(apriltag)

# # Fetch allwpilib
FetchContent_Declare(
  allwpilib
  GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
  GIT_TAG        v2026.2.1
)

FetchContent_MakeAvailable(allwpilib)

find_package(OpenCV REQUIRED)

add_subdirectory(src)