#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/StructTopic.h>
#include <vector>
#include "src/camera/CameraStream.h" 
#include "src/camera/CameraConfig.h"
#include "src/utils/PCH.h"
#include "src/yolo/Yolo.h"

namespace gamepiece {

// 2026: single gamepiece "fuel"
void run_fuel_detect(yolo::Yolo& model,
                     const std::vector<std::string>& class_names,
                     const camera::camera_config_t& config,
                     nt::StructTopic<frc::Pose2d>& fuel_topic,
                     bool debug);

void run_fuel_detect_no_img(yolo::Yolo& model,
                            const std::vector<std::string>& class_names);

}  // namespace gamepiece