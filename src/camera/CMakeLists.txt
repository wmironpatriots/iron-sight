add_library(camera CameraCv.cc CameraStream.cc) 
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} wpilibc nlohmann_json::nlohmann_json utils)
