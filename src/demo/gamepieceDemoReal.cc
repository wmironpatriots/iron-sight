// File: gamepiece_detector_demo.cc
// Purpose: Demo for testing GamepieceDetector with CameraIOCv + YOLO

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/gamepiece/gamepieceVector.h"
#include "src/utils/NtUtils.h"
#include "src/utils/PCH.h"
#include "src/yolo/ModelConstants.h"
#include "src/yolo/Yolo.h"

namespace {
auto PumpGuiEventsAndGetKey() -> int {
  return cv::waitKey(1);
}
}  // namespace

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    "0",
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    100,
    frc::Transform3d{
    frc::Translation3d{0.25_m, 0.0_m, 0.6_m},
    frc::Rotation3d{0_rad, units::degree_t{45}, 0_rad}},
    camera::camera_intrinsics_t{619.9026951017695, 372.2792812903024,
                                539.3898006061588, 539.0735059998198,
                                -0.3241468189388152, 0.10782527225392564,
                                -0.016410664585260946, 0.00013396458313481827,
                                -0.0002794187083645791}};

auto main() -> int {
  camera::CameraIOCv camera(kDemoCam);

  const yolo::module_config_t model_cfg = yolo::kAlphaModel;
  yolo::Yolo model(model_cfg.path, model_cfg.swap_rb, /*verbose=*/true);

  const std::string window_name = "Gamepiece Detector Demo";
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

  int print_count = 0;

  while (true) {
    const auto start = std::chrono::high_resolution_clock::now();

    camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
    if (tframe.frame.empty()) {
      std::cerr << "[GamepieceDetector] Empty frame\n";
      continue;
    }

    // Raw detections for drawing
    cv::Mat out = model.RunModel(tframe.frame);

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    model.Postprocess(tframe.frame.rows, tframe.frame.cols, out, boxes,
                      confidences, class_ids);

    // Position-estimated detections
    const std::vector<gamepiece::GamepieceDetection> detections =
        gamepiece::DetectGamepieces(model, tframe.frame, kDemoCam);

    cv::Mat annotated = tframe.frame.clone();
    yolo::Yolo::DrawDetections(annotated, boxes, class_ids, confidences,
                               model_cfg.classes);

    // Draw estimated robot-relative positions next to each box if possible
    const size_t count = std::min(boxes.size(), detections.size());
    for (size_t i = 0; i < count; ++i) {
      const auto& box = boxes[i];
      const auto& det = detections[i];

      const std::string pos_label =
          cv::format("(%.2f, %.2f)m",
                     det.position.X().value(),
                     det.position.Y().value());

      const cv::Point text_pt(box.x,
                              std::min(box.y + box.height + 18,
                                       annotated.rows - 5));

      cv::putText(annotated, pos_label, text_pt, cv::FONT_HERSHEY_SIMPLEX,
                  0.5, cv::Scalar(255, 0, 0), 1);
    }

    cv::Mat fixed;
    cv::cvtColor(annotated, fixed, cv::COLOR_BGR2RGB);

    const auto end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed = end - start;

    if (!detections.empty()) {
      if (print_count >= 1) {
        std::system("clear");
        std::cerr << "FPS: " << (1.0 / elapsed.count()) << "\n";
        std::cerr << "Raw YOLO boxes: " << boxes.size() << "\n";
        std::cerr << "Gamepiece positions: " << detections.size() << "\n";

        const auto& first = detections[0];
        std::cerr << "First gamepiece position (robot-relative): ("
                  << first.position.X().value() << ", "
                  << first.position.Y().value() << ") m\n";
        std::cerr << "First confidence: " << first.confidence << "\n";

        if (!boxes.empty()) {
          const auto& b = boxes[0];
          const double cx = b.x + b.width / 2.0;
          const double cy = b.y + b.height / 2.0;

          std::cerr << "First box center px: (" << cx << ", " << cy << ")\n";
          std::cerr << "First box w,h: (" << b.width << ", " << b.height
                    << ")\n";

          constexpr double kHFovRad = 70.0 * (std::numbers::pi / 180.0);
          const double yaw =
              yolo::Yolo::GetObjectAngle(cx, kHFovRad, tframe.frame.cols);
          std::cerr << "Approx yaw (rad): " << yaw << "\n";
        }

        print_count = 0;
      }
      print_count++;
    }

    cv::imshow(window_name, fixed);

    const int key = PumpGuiEventsAndGetKey();
    if (key == 'q' || key == 27) {
      break;
    }
  }

  return 0;
}