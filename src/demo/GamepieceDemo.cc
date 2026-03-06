// File: GamepieceDemo.cc
// Purpose: Run gamepiece::run_fuel_detect locally (no NT server/client started).

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>

#include <iostream>
#include <string>
#include <vector>

#include "src/camera/CameraConfig.h"
#include "src/gamepiece/gamepiece.h"
#include "src/yolo/ModelConstants.h"
#include "src/yolo/Yolo.h"

// Same camera config pattern as your multitag / yolo demo
inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    "/dev/video0",
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    100,

    
    frc::Transform3d(
        frc::Translation3d{units::meter_t{0.0},
                           units::meter_t{0.0},
                           units::meter_t{0.65}},  
        frc::Rotation3d{0_rad, 0_rad, 0_rad}),

    camera::camera_intrinsics_t{619.9026951017695, 372.2792812903024,
                                539.3898006061588, 539.0735059998198,
                                -0.3241468189388152, 0.10782527225392564,
                                -0.016410664585260946, 0.00013396458313481827,
                                -0.0002794187083645791}};
auto main() -> int {
  const yolo::module_config_t model_cfg = yolo::kAlphaModel;

  // model_cfg.path is std::string in your codebase
  std::cout << "[GamepieceDemo] model path = '" << model_cfg.path << "'\n";
  if (model_cfg.path.empty()) {
    std::cerr
        << "[GamepieceDemo] ERROR: yolo::kAlphaModel.path is empty.\n"
        << "Fix src/yolo/ModelConstants.h and set kAlphaModel.path to your .onnx file.\n";
    return 1;
  }

  // Construct model exactly like your yolo_demo
  yolo::Yolo model(model_cfg.path, model_cfg.swap_rb, /*verbose=*/true);

  // Use classes from the model config (same as yolo_demo)
  const std::vector<std::string> class_names = model_cfg.classes;

  // NT topic exists, but we do NOT start server/client
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto fuel_topic = inst.GetStructTopic<frc::Pose2d>("/gamepiece/fuel_pose");

  const bool debug = true;
  std::cout << "[GamepieceDemo] Starting run_fuel_detect (no NT server/client)...\n";

  gamepiece::run_fuel_detect(model, class_names, kDemoCam, fuel_topic, debug);
  return 0;
}