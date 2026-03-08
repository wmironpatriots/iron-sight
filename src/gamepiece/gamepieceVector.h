#pragma once
#include "src/yolo/Yolo.h"
#include "src/camera/CameraConfig.h"
#include "src/utils/PCH.h"

namespace gamepiece {

struct GamepieceDetection {
  frc::Translation2d position;
  float confidence;
};

std::vector<GamepieceDetection> DetectGamepieces(
    yolo::Yolo& model,
    const cv::Mat& frame,
    const camera::camera_config_t& config);

}