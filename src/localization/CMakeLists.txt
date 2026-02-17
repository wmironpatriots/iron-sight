add_library(localization CvAprilTagSearcher.cc) 
target_link_libraries(localization PUBLIC wpilibc apriltag nlohmann_json::nlohmann_json OpenCV camera utils)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(localization PUBLIC AprilTagSearcher.h CvAprilTagSearcher.h MultiTagPositionEstimator.h PnpDistanceTrigSolverPositionEstimator.h PositionEstimator.h)
endif()