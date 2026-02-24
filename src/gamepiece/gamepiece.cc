#include "gamepiece.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/StructTopic.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include "src/camera/CameraStream.h"
#include "src/utils/PCH.h"
#include "src/yolo/Yolo.h"

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
                     camera::CameraStream& stream,
                     nt::StructTopic<frc::Pose2d>& fuel_topic,
                     const nlohmann::json& intrinsics,
                     const nlohmann::json& extrinsics, bool debug) {
  nt::StructPublisher<frc::Pose2d> fuel_pub = fuel_topic.Publish();
  cv::Mat color;
  std::vector<cv::Rect> bboxes(MAX_DETECTIONS);
  std::vector<float> confidences(MAX_DETECTIONS);
  std::vector<int> class_ids(MAX_DETECTIONS);
  const float pinhole_height = extrinsics["translation_z"];
  const float cam_cx = intrinsics["cx"];
  const float cam_cy = intrinsics["cy"];
  const float focal_length_vertical = intrinsics["fy"].get<float>();
  const float focal_length_horizontal = intrinsics["fx"].get<float>();
  const float cam_pitch = extrinsics["rotation_y"];
  const frc::Pose3d cam_pose{
      frc::Translation3d{
          units::meter_t{extrinsics["translation_x"].get<float>()},
          units::meter_t{extrinsics["translation_y"].get<float>()},
          units::meter_t{extrinsics["translation_z"].get<float>()}},
      frc::Rotation3d{units::radian_t{extrinsics["rotation_x"].get<float>()},
                      units::radian_t{extrinsics["rotation_y"].get<float>()},
                      units::radian_t{extrinsics["rotation_z"].get<float>()}}};

  frc::Transform3d target_pose_cam_relative;
  frc::Pose3d target_pose_robot_relative;

  while (true) {
    color = stream.getFrame();
    if (color.empty()) {
      if (debug)
        std::cout << "Empty frame\n";
      continue;
    }

    mutex.lock();
    cv::Mat out = model.RunModel(color);
    model.Postprocess(color.rows, color.cols, out, bboxes, confidences,
                      class_ids);
    mutex.unlock();
    for (size_t i = 0; i < MAX_DETECTIONS; i++) {
      const cv::Rect& box = bboxes[i];
      if (box.width <= 0 || box.height <= 0)
        continue;

      const int c_y = box.y + box.height / 2;
      const int c_x = box.x + box.width / 2;

      const float cam_relative_pitch =
          std::atan2((float)c_x - cam_cx, focal_length_vertical);
      const float phi = cam_relative_pitch + cam_pitch;
      const float distance = pinhole_height / std::sin(phi);

      const float cam_relative_yaw =
          std::atan2((float)c_x - cam_cx, focal_length_horizontal);
      target_pose_cam_relative = {
          frc::Translation3d{
              units::meter_t{distance * std::cos(cam_relative_pitch) *
                             std::cos(cam_relative_yaw)},
              units::meter_t{distance * std::cos(cam_relative_pitch) *
                             std::sin(cam_relative_yaw)},
              units::meter_t{distance * -std::sin(cam_relative_pitch)}},
          frc::Rotation3d{0_rad, units::radian_t{cam_relative_pitch},
                          units::radian_t{-cam_relative_yaw}}};
      target_pose_robot_relative =
          cam_pose.TransformBy(target_pose_cam_relative);
      fuel_pub.Set(target_pose_robot_relative.ToPose2d());
      if (debug) {
        int cid = (i < class_ids.size()) ? class_ids[i] : 0;
        std::string name = (cid >= 0 && cid < (int)class_names.size())
                               ? class_names[cid]
                               : "fuel";
        std::cout << "Detected " << name << " dist=" << distance
                  << " conf=" << confidences[i] << "\n";
        std::cout << "\tc_y:\t" << c_y << "\tcam_relative_pitch:\t"
                  << cam_relative_pitch << "\tphi:\t" << phi << "\tyaw:\t"
                  << cam_relative_yaw << "\n";
        std::cout << "TargetPose: " << target_pose_cam_relative << "\n";
        std::cout << "Robot_relative: " << target_pose_robot_relative << "\n";
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
    if (box.width <= 0 || box.height <= 0)
      continue;

    int cid = (i < class_ids.size()) ? class_ids[i] : 0;
    std::string name =
        (cid >= 0 && cid < (int)class_names.size()) ? class_names[cid] : "fuel";
    std::cout << "Detected " << name << " conf=" << confidences[i] << " box=("
              << box.x << "," << box.y << "," << box.width << "," << box.height
              << ")\n";
  }
}
}  // namespace gamepiece