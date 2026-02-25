add_executable(image_collector ImageCollector.cc)
target_link_libraries(image_collector PRIVATE ${OpenCV_LIBS} camera wpilibc utils)

add_library(yolo Yolo.cc)
target_link_libraries(yolo PRIVATE ${OpenCV_LIBS} utils)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(camera PUBLIC Camera.h CameraCv.h CameraStream.h)
endif()