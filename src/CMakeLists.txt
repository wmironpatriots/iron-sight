cmake_minimum_required(VERSION 3.15)

include_directories(${CMAKE_SOURCE_DIR})

if(ENABLE_CLANG_TIDY)
    set(CMAKE_CXX_CLANG_TIDY
        clang-tidy;
        -checks=modernize-*,performance-*;
        -warnings-as-errors=*;
    )
endif()

add_subdirectory(test)
add_subdirectory(utils)
add_subdirectory(camera)
add_subdirectory(localization)

# Binary for center cameras coproc
add_executable(center center.cc)
target_link_directories(center PRIVATE ${OpenCV_LIBS} apriltag ntcore nlohmann_json::nlohmann_json)
set_target_properties(center PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

# Binary for side cameras coproc
add_executable(side side.cc)
target_link_directories(side PRIVATE ${OpenCV_LIBS} apriltag ntcore nlohmann_json::nlohmann_json)
set_target_properties(side PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

set(CMAKE_CXX_CLANG_TIDY "")
