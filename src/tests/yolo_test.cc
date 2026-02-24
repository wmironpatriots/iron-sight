#include "src/yolo/Yolo.h"
#include "src/yolo/ModelConstants.h"  

#include "src/camera/CameraCv.h"
#include "src/camera/CameraStream.h"

#include <iostream>
#include <numbers>
#include <opencv2/opencv.hpp>
#include <string>

const int MAX_DETECTIONS = 10;

auto main() -> int {
  const yolo::module_config_t& cfg = yolo::kAlphaModel;

  std::cout << "Loading model: " << cfg.path << "\n";

  yolo::Yolo model(cfg.path, cfg.swap_rb, true);

  camera::CameraConfig cam_cfg;
  cam_cfg.deviceId = 0;
  cam_cfg.apiId = cv::CAP_ANY;

  auto cam = std::make_unique<camera::CameraCv>(cam_cfg);
  camera::CameraStream stream(std::move(cam));

  const std::string kWin = "Alpha YOLO Test";
  cv::namedWindow(kWin, cv::WINDOW_NORMAL);

  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);

  std::cout << "Running detection... press 'q' to exit\n";

  while (true) {
    cv::Mat frame = stream.getFrame();
    if (frame.empty()) {
      std::cerr << "Camera returned empty frame\n";
      continue;
    }

    cv::Mat detections = model.RunModel(frame);

    model.Postprocess(frame.rows, frame.cols, detections,
                      bboxes, confidences, class_ids);

    
    yolo::Yolo::DrawDetections(frame, bboxes, class_ids,
                               confidences, cfg.classes);

    
    if (!detections.empty() && detections.cols >= 4) {
      float cx = (detections.at<float>(0, 0) +
                  detections.at<float>(0, 2)) / 2.0f;

      std::cout << "Object angle: "
                << yolo::Yolo::GetObjectAngle(
                       cx, std::numbers::pi * (3.0 / 4.0))
                << "\n";
    }

    
    cv::imshow(kWin, frame);

    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }

  return 0;
}