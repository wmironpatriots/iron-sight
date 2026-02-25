// File: yolo_demo.cc
// Purpose: YOLO demo using your CameraIOCv + YOLO wrapper (similar loop style to the multitag demo)

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/utils/NtUtils.h"
#include "src/utils/PCH.h"
#include "src/yolo/ModelConstants.h"
#include "src/yolo/Yolo.h"

namespace {
auto PumpGuiEventsAndGetKey() -> int {
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
  return cv::pollKey();
#else
  return cv::waitKey(1);
#endif
}
}  // namespace

// Same camera config pattern as your multitag demo
inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    0,
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    120,
    frc::Transform3d(),
    camera::camera_intrinsics_t{619.9026951017695, 372.2792812903024,
                                539.3898006061588, 539.0735059998198,
                                -0.3241468189388152, 0.10782527225392564,
                                -0.016410664585260946, 0.00013396458313481827,
                                -0.0002794187083645791}};

auto main() -> int {
  // Camera
  camera::CameraIOCv camera(kDemoCam);

  // NetworkTables (optional; keep if you like the pattern)
  utils::StartNetworkTables(true);

  // YOLO model config (from your ModelConstants.h)
  const yolo::module_config_t model_cfg = yolo::kAlphaModel;

  // NOTE: model_cfg.path is whatever you set (currently "iron-sight/models/best.onnx")
  // Make sure you run from a working directory where that relative path exists,
  // OR change model_cfg.path to an absolute path.
  yolo::Yolo model(model_cfg.path, model_cfg.swap_rb, /*verbose=*/true);

  const std::string windowName = "YOLO Detection Demo";
  cv::namedWindow(windowName, cv::WINDOW_NORMAL);

  int printCount = 0;

  while (true) {
    const auto start = std::chrono::high_resolution_clock::now();

    camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
    if (tframe.frame.empty()) {
      std::cerr << "[YOLO] Empty frame\n";
      continue;
    }

    cv::Mat out = model.RunModel(tframe.frame);

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    model.Postprocess(tframe.frame.rows, tframe.frame.cols, out, boxes,
                      confidences, class_ids);


    cv::Mat annotated = tframe.frame.clone();
    yolo::Yolo::DrawDetections(annotated, boxes, class_ids, confidences,
                               model_cfg.classes);

    // Example: print FPS + first detection center (like your console output style)
    const auto end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> elapsed = end - start;

    if (!boxes.empty()) {
      if (printCount >= 1) {
        std::system("clear");
        std::cerr << "FPS: " << (1.0 / elapsed.count()) << "\n";
        std::cerr << "Detections: " << boxes.size() << "\n";

        // First box center
        const auto& b = boxes[0];
        const double cx = b.x + b.width / 2.0;
        const double cy = b.y + b.height / 2.0;

        std::cerr << "First box center px: (" << cx << ", " << cy << ")\n";
        std::cerr << "First box w,h: (" << b.width << ", " << b.height << ")\n";
        std::cerr << "Conf: " << confidences[0] << "\n";

        // If you know your camera horizontal FOV (radians), you can compute angle:
        // Example placeholder: 70 degrees -> 1.22173 rad. Replace with your real HFOV.
        constexpr double kHFovRad = 70.0 * (std::numbers::pi / 180.0);
        const double yaw =
            yolo::Yolo::GetObjectAngle(cx, kHFovRad, tframe.frame.cols);
        std::cerr << "Approx yaw (rad): " << yaw << "\n";

        printCount = 0;
      }
      printCount++;
    }

    cv::imshow(windowName, annotated);

    const int key = PumpGuiEventsAndGetKey();
    if (key == 'q' || key == 27) {  // q or ESC
      break;
    }
  }

  return 0;
}