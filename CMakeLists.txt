cmake_minimum_required(VERSION 3.15)
project(IRONSIGHT CXX C)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # enable o3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # warn all

option(ENABLE_CLANG_TIDY "Enable clang-tidy for static analysis" ON)

include(FetchContent)

find_package(OpenCV REQUIRED)

# Fetch Json
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.12.0  # Or a specific hash for stability
)

FetchContent_MakeAvailable(json)

# Fetch allwpilib
FetchContent_Declare(
  allwpilib
  GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
  GIT_TAG        v2026.2.1
)

FetchContent_MakeAvailable(allwpilib)

# Fetch apriltag
# FetchContent_Declare(
#   apriltag
#   GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
#   GIT_TAG        v3.4.5
# )

# FetchContent_MakeAvailable(apriltag)

add_subdirectory(src)