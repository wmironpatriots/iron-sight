// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: Camera.h
// Purpose: Define camera structs & base camera class
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/utils/PCH.h"

namespace camera {
/** Represents a camera  */
struct CameraConfig {
  /** camera id */
  int deviceId;
  /** API backend to use */
  int apiId;
};

/** Represents a camera frame recorded at a specified timestamp */
struct TimestampedFrame {
  /** Represents the recorded frame */
  cv::Mat frame;
  /** Timestamp representing when frame was captured */
  units::second_t timestamp;
};

/** Hardware interface for interacting with a camera */
class Camera {
 public:
  virtual ~Camera() = default;
  /** @return dense matrix representing raw recorded frame */
  virtual auto getFrame() -> cv::Mat;
  /** returns TimestampedFrame representing the frame recorded at a specific timestamp*/
  virtual auto getTimestampedFrame() -> TimestampedFrame;

 private:
  cv::VideoCapture mCapture;
};
}  // namespace camera