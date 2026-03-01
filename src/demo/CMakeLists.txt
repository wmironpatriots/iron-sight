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
target_link_libraries(tag_tracker_demo PRIVATE ${OpenCV_LIBS} wpilibc apriltag ntcore localization camera utils)

# Demo executable for MultiTagPositionEstimator
add_executable(multitag_estimation_demo MultiTagEstimationDemo.cc)
target_link_libraries(multitag_estimation_demo PRIVATE ${OpenCV_LIBS} wpilibc apriltag ntcore localization camera utils)

# Demo executable for FPSDemo
add_executable(FPSDemo FPSDemo.cc)
target_link_libraries(FPSDemo PRIVATE ${OpenCV_LIBS} wpilibc apriltag ntcore localization camera utils)

# Demo executable for YOLO detection
#add_executable(YoloDemo YoloDemo.cc)
#target_link_libraries(YoloDemo PRIVATE ${OpenCV_LIBS} wpilibc ntcore localization camera utils yolo)

set(CMAKE_CXX_CLANG_TIDY "")