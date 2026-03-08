add_library(gamepiece gamepieceVector.cc)
target_link_libraries(gamepiece wpilibc ntcore ${OpenCV_LIBS} camera utils yolo)