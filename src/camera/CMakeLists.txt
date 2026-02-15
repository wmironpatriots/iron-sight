add_library(camera CameraCv.cc CameraStream.cc) 
target_link_libraries(camera PUBLIC wpilibc nlohmann_json::nlohmann_json OpenCV utils)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(camera PUBLIC Camera.h CameraIOcv.h CameraStream.h)
endif()