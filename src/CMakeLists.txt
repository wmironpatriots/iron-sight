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

# Binary for center cameras coproc
add_executable(u1 u1_main.cc)
target_link_directories(u1 PRIVATE ${OpenCV_LIBS} apriltag ntcore nlohmann_json::nlohmann_json utils)
# set_target_properties(u1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_DIR}/bin")

# Binary for side cameras coproc
add_executable(u2 u2_main.cc)
target_link_directories(u2 PRIVATE ${OpenCV_LIBS} apriltag ntcore nlohmann_json::nlohmann_json utils)
# set_target_properties(u2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_DIR}/bin")

# Demo executable for CvAprilTagSearcher
add_executable(cvapriltagsearcher_demo CvAprilTagSearcherDemo.cc)
target_link_libraries(cvapriltagsearcher_demo PRIVATE ${OpenCV_LIBS} wpilibc apriltag ntcore localization camera utils)

set(CMAKE_CXX_CLANG_TIDY "")
