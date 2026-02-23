// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraCv.cc
// Purpose: Define base camera hardware interface
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/camera/CameraCv.h"

namespace camera {
CameraCv::CameraCv(const CameraConfig& config) {
  mConfig = config;
  mCameraCapture = cv::VideoCapture(config.deviceId, config.apiId);

  // Check if stream connected to camera
  if (!mCameraCapture.isOpened()) {
    printf("Error ~ Failed to open camera\n");
    throw;
  }
}

auto CameraCv::getTimestampedFrame() -> TimestampedFrame {
  TimestampedFrame tframe;
  mCameraCapture.read(tframe.frame);
  tframe.timestamp = frc::Timer::GetFPGATimestamp();

  return tframe;
}
}  // namespace camera
