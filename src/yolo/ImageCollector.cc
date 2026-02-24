// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: ImageCollector.cc
// Purpose: Collect many images for data set creation
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/Camera.h"
#include "src/camera/CameraIOCv.h"
#include "src/utils/PCH.h"

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    2,
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    120,
    frc::Transform3d(),
    camera::camera_intrinsics_t{
        619.9026951017695,
        372.2792812903024,
        539.3898006061588,
        539.0735059998198,
        -0.3241468189388152,
        0.10782527225392564,
        -0.016410664585260946,
        0.00013396458313481827,
        -0.0002794187083645791
    }
};

auto main() -> int {
  std::string img_dir = "iron-sight/logs/collected_imgs";
  std::filesystem::create_directories(img_dir);
  

  camera::CameraIOCv cam(kDemoCam);
  while (true) {
    camera::timestamped_frame_t tframe = cam.GetTimestampedFrame();

    if (!tframe.frame.empty()) {
      const std::string path =
          img_dir + std::to_string(tframe.timestamp.value()) + ".png";
      cv::imwrite(path, tframe.frame);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }
}