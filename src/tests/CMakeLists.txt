add_executable(yolo_test
    src/yolo/yolo_test.cc
)
target_link_libraries(yolo_test PRIVATE
    ${OpenCV_LIBS}
)