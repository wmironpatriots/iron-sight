add_library(yolo Yolo.cc)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS} utils)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(camera PUBLIC Camera.h CameraCv.h CameraStream.h)
endif()