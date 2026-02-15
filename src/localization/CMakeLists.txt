add_library(localization AprilTagDetector.cc) 
target_link_libraries(localization PUBLIC wpilibc nlohmann_json::nlohmann_json OpenCV utils)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(localization PUBLIC AprilTagDetector.h)
endif()