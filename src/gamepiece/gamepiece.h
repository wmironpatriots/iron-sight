#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/StructTopic.h>
#include <nlohmann/json.hpp>

#include "src/camera/CameraStream.h"  
#include "src/yolo/Yolo.h"

namespace gamepiece {

    
void run_fuel_detect(yolo::Yolo& model,
                     const std::vector<std::string>& class_names,
                     camera::CameraStream& stream,
                     nt::StructTopic<frc::Pose2d>& fuel_topic,
                     const nlohmann::json& intrinsics,
                     const nlohmann::json& extrinsics,
                     bool debug);


void run_fuel_detect_no_img(yolo::Yolo& model,
                            const std::vector<std::string>& class_names);

}  // namespace gamepiece