#include "src/utils/PCH.h"
#include "src/yolo/Yolo.h"
#include "src/camera/CameraConfig.h"
#include "src/gamepiece/gamepieceVector.h"

namespace gamepiece {

auto DetectGamepieces(yolo::Yolo& model,
                      const cv::Mat& frame,
                      const camera::camera_config_t& config)
    -> std::vector<GamepieceDetection> {

  std::vector<GamepieceDetection> detections;

  if (frame.empty()) {
    return detections;
  }

  // Run inference
  cv::Mat out = model.RunModel(frame);

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;

  model.Postprocess(frame.rows, frame.cols, out, boxes, confidences, class_ids);

  // Camera intrinsics
  const float cx = config.intrinsics_calibration.cx;
  const float cy = config.intrinsics_calibration.cy;
  const float fx = config.intrinsics_calibration.fx;
  const float fy = config.intrinsics_calibration.fy;

  
  const float cam_x = config.transform_wrt_chassis.X().value();
  const float cam_y = config.transform_wrt_chassis.Y().value();
  const float cam_z = config.transform_wrt_chassis.Z().value();

  
  const float cam_pitch = config.transform_wrt_chassis.Rotation().value();

  for (size_t i = 0; i < boxes.size(); i++) {

    const cv::Rect& box = boxes[i];
    if (box.width <= 0 || box.height <= 0) continue;

    // Center pixel
    const float px = box.x + box.width * 0.5f;
    const float py = box.y + box.height * 0.5f;

    // Pixel → ray angles
    const float rel_pitch = std::atan2(py - cy, fy);
    const float rel_yaw   = std::atan2(px - cx, fx);

    const float total_pitch = cam_pitch + rel_pitch;

    const float tan_pitch = std::tan(total_pitch);
    if (std::abs(tan_pitch) < 1e-4f) continue;

    
    const float forward = cam_z / tan_pitch;
    if (forward <= 0) continue;

    const float lateral = forward * std::tan(rel_yaw);

    const float x = cam_x + forward;
    const float y = cam_y + lateral;

    GamepieceDetection detection;

    detection.position = frc::Translation2d(
        units::meter_t{x},
        units::meter_t{y});

    detection.confidence = confidences[i];

    detections.push_back(detection);
  }

  return detections;
}

}  // namespace gamepiece