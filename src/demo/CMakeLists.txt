cmake_minimum_required(VERSION 3.15)

include_directories(${CMAKE_SOURCE_DIR})

if(ENABLE_CLANG_TIDY)
    set(CMAKE_CXX_CLANG_TIDY
        clang-tidy;
        -checks=modernize-*,performance-*;
        -warnings-as-errors=*;
    )
endif()

# Demo executable for CvAprilTagSearcher
add_executable(tag_tracker_demo TagTrackingDemo.cc)
target_link_libraries(cvapriltagsearcher_demo PRIVATE ${OpenCV_LIBS} wpilibc apriltag ntcore localization camera utils)

# Demo executable for MultiTagPositionEstimator
add_executable(multitag_estimation_demo MultiTagEstimationDemon.cc)
target_link_libraries(multitagpositionestimater_demo PRIVATE ${OpenCV_LIBS} wpilibc apriltag ntcore localization camera utils)

set(CMAKE_CXX_CLANG_TIDY "")