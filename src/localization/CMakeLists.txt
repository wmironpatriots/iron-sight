add_library(localization CvAprilTagSearcher.cc MultiTagPositionEstimator.cc)
target_link_libraries(localization PUBLIC ${OpenCV_LIBS} wpilibc apriltag nlohmann_json::nlohmann_json camera utils)