add_library(camera CameraIOCv.cc CameraStream.cc) 
target_link_libraries(camera PUBLIC ${OpenCV_LIBS} wpilibc utils)
