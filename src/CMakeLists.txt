cmake_minimum_required(VERSION 3.15)

include_directories(${CMAKE_SOURCE_DIR})

if(ENABLE_CLANG_TIDY)
    set(CMAKE_CXX_CLANG_TIDY
        clang-tidy;
        -checks=modernize-*,performance-*;
        -warnings-as-errors=*;
    )
endif()

add_subdirectory(camera)
add_subdirectory(utils)
add_subdirectory(localization)
add_subdirectory(demo)
# add_subdirectory(yolo)

# Binary for center cameras coproc
add_executable(u1 u1_main.cc)
target_link_libraries(u1 PRIVATE ${OpenCV_LIBS} apriltag ntcore nlohmann_json::nlohmann_json localization camera utils)
set_target_properties(u1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# Binary for side cameras coproc
add_executable(u2 u2_main.cc)
target_link_libraries(u2 PRIVATE ${OpenCV_LIBS} apriltag ntcore nlohmann_json::nlohmann_json localization camera utils)
set_target_properties(u2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

set(CMAKE_CXX_CLANG_TIDY "")
