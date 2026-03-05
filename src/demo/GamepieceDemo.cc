// File: gamepiece_detect_demo.cc
// Purpose: Run gamepiece::run_fuel_detect locally (no NT server/client started).

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>

#include <iostream>
#include <string>
#include <vector>

#include "src/gamepiece/gamepiece.h"
#include "src/camera/CameraConfig.h"
#include "src/yolo/Yolo.h"
#include "src/yolo/ModelConstants.h"

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    0,
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    100,
    frc::Transform3d(),
    camera::camera_intrinsics_t{619.9026951017695, 372.2792812903024,
                                539.3898006061588, 539.0735059998198,
                                -0.3241468189388152, 0.10782527225392564,
                                -0.016410664585260946, 0.00013396458313481827,
                                -0.0002794187083645791}};

static std::vector<std::string> MakeClassNames() {
  // Must match your model classes
  return {"fuel"};
}

int main(int argc, char** argv) {
const yolo::module_config_t model_cfg = yolo::kAlphaModel;
  std::string model_path = model_cfg.path;
  if (argc > 1) model_path = argv[1];

  // Your Yolo.h constructor: Yolo(model_path, swap_rb, verbose)
  const bool swap_rb = true;
  const bool verbose = true;
  yolo::Yolo model(model_path, swap_rb, verbose);

  auto cfg = MakeDemoCameraConfig();
  auto class_names = MakeClassNames();

  // NetworkTables instance exists, but we do NOT start server/client.
  // This keeps your run_fuel_detect signature unchanged.
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto fuel_topic = inst.GetStructTopic<frc::Pose2d>("/gamepiece/fuel_pose");

  const bool debug = true;
  std::cout << "Starting gamepiece fuel detect (no NT server/client)...\n";

  // Runs forever
  gamepiece::run_fuel_detect(model, class_names, cfg, fuel_topic, debug);
  return 0;
}