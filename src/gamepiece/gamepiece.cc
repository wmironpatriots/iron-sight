#include "gamepiece.h"

namespace gamepiece {
static constexpr int MAX_DETECTIONS = 6;
static std::mutex mutex;

auto operator<<(std::ostream& os, const frc::Pose3d& p) -> std::ostream& {
  os << "Point(" << p.X().value() << ", " << p.Y().value() << ", "
     << p.Z().value() << ")" << "\nRotation:\nPitch:\t"
     << p.Rotation().Y().value() << "\nRoll:\t" << p.Rotation().X().value()
     << "\nYaw:\t" << p.Rotation().Z().value() << ")";
  return os;
}

auto operator<<(std::ostream& os, const frc::Transform3d& p) -> std::ostream& {
  os << "Point(" << p.X().value() << ", " << p.Y().value() << ", "
     << p.Z().value() << ")" << "\nRotation:\nPitch:\t"
     << p.Rotation().Y().value() << "\nRoll:\t" << p.Rotation().X().value()
     << "\nYaw:\t" << p.Rotation().Z().value() << ")";
  return os;
}

void run_fuel_detect(yolo::Yolo& model,
                     const std::vector<std::string>& class_names,
                     const camera::camera_config_t& config,
                     nt::StructTopic<frc::Pose2d>& fuel_topic,
                     bool debug) {
  camera::CameraIOCv stream(config);

  // Uncomment if you want to publish to NT
  // nt::StructPublisher<frc::Pose2d> fuel_pub = fuel_topic.Publish();

  cv::Mat color;

  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);

  // Camera height above floor (meters). Using camera Z from the mount transform.
  const float pinhole_height =
      static_cast<float>(config.transform_wrt_chassis.Z().value());

  // Intrinsics
  const auto cam_cx = static_cast<float>(config.intrinsics_calibration.cx);
  const auto cam_cy = static_cast<float>(config.intrinsics_calibration.cy);
  const auto fx = static_cast<float>(config.intrinsics_calibration.fx);
  const auto fy = static_cast<float>(config.intrinsics_calibration.fy);

  // Camera pitch (radians). TODO: replace with real mounted pitch.
  const units::radian_t cam_pitch = units::degree_t{45};

  // Robot->Camera transform
  const frc::Transform3d cam_pose = config.transform_wrt_chassis;

  frc::Transform3d target_pose_cam_relative;
  frc::Transform3d target_pose_robot_relative;

  while (true) {
    color = stream.GetTimestampedFrame().frame;
    if (color.empty()) {
      if (debug) std::cout << "Empty frame\n";
      continue;
    }

    // Run model + postprocess
    mutex.lock();
    cv::Mat out = model.RunModel(color);
    model.Postprocess(color.rows, color.cols, out, bboxes, confidences,
                      class_ids);
    mutex.unlock();

    for (size_t i = 0; i < MAX_DETECTIONS; i++) {
      const cv::Rect& box = bboxes[i];
      if (box.width <= 0 || box.height <= 0) continue;

      const int c_x = box.x + box.width / 2;
      const int c_y = box.y + box.height / 2;

      // Angles from pixel center (pinhole model)
      // Pitch uses vertical offset (y), yaw uses horizontal offset (x)
      const float cam_relative_pitch =
          std::atan2(static_cast<float>(c_y) - cam_cy, fy);
      const float cam_relative_yaw =
          std::atan2(static_cast<float>(c_x) - cam_cx, fx);

      // Combine pitch with camera mounting pitch
      const float phi =
          cam_relative_pitch + static_cast<float>(cam_pitch.value());

      // Distance estimate (simple trig)
      const float distance = pinhole_height / std::sin(phi);

      // Build camera->target transform
      target_pose_cam_relative = frc::Transform3d{
          frc::Translation3d{
              units::meter_t{distance * std::cos(cam_relative_pitch) *
                             std::cos(cam_relative_yaw)},
              units::meter_t{distance * std::cos(cam_relative_pitch) *
                             std::sin(cam_relative_yaw)},
              units::meter_t{distance * -std::sin(cam_relative_pitch)}},
          frc::Rotation3d{0_rad, units::radian_t{cam_relative_pitch},
                          units::radian_t{-cam_relative_yaw}}};

      // Robot->Target = (Robot->Camera) + (Camera->Target)
      target_pose_robot_relative = cam_pose + target_pose_cam_relative;

      // Publish if desired (convert Transform3d -> Pose3d -> Pose2d)
      // frc::Pose3d robot_pose = frc::Pose3d{}.TransformBy(target_pose_robot_relative);
      // fuel_pub.Set(robot_pose.ToPose2d());

      if (debug) {
        const size_t cid = class_ids[i];
        const std::string name =
            (cid < class_names.size())
                ? class_names[cid]
                : "fuel";

        std::cout << "Detected " << name << " dist=" << distance
                  << " conf=" << confidences[i] << "\n";
        std::cout << "\tc_x:\t" << c_x << "\tc_y:\t" << c_y
                  << "\tcam_rel_pitch:\t" << cam_relative_pitch
                  << "\tphi:\t" << phi << "\tyaw:\t" << cam_relative_yaw
                  << "\n";
        std::cout << "TargetPose (cam): " << target_pose_cam_relative << "\n";
        std::cout << "TargetPose (robot): " << target_pose_robot_relative
                  << "\n";
      }
    }
  }
}

void run_fuel_detect_no_img(yolo::Yolo& model,
                            const std::vector<std::string>& class_names) {
  std::vector<cv::Rect> bboxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;

  cv::Mat color(640, 640, CV_8UC3);

  mutex.lock();
  cv::Mat out = model.RunModel(color);
  model.Postprocess(color.rows, color.cols, out, bboxes, confidences,
                    class_ids);
  mutex.unlock();

  if (bboxes.empty()) {
    std::cout << "No detections\n";
    return;
  }

  const size_t count = std::min<size_t>(bboxes.size(), MAX_DETECTIONS);
  for (size_t i = 0; i < count; i++) {
    const cv::Rect& box = bboxes[i];
    if (box.width <= 0 || box.height <= 0) continue;

    const size_t cid = class_ids[i];
    const std::string name =
        (cid < class_names.size())
            ? class_names[cid]
            : "fuel";

    std::cout << "Detected " << name << " conf=" << confidences[i]
              << " box=(" << box.x << "," << box.y << "," << box.width << ","
              << box.height << ")\n";
  }
}
}  // namespace gamepiece