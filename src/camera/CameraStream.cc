// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraStream.cc
// Purpose: Define
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/camera/CameraStream.h"

namespace camera {
CameraStream::CameraStream(std::unique_ptr<Camera> io) : mIO(std::move(io)) {
  mTimestampedFrame = mIO->getTimestampedFrame();

  mThread = std::thread([this]() -> void {
    TimestampedFrame tframe;
    tframe = mIO->getTimestampedFrame();

    mMutex.lock();
    mTimestampedFrame = tframe;
    mMutex.unlock();
  });
};

auto CameraStream::getTimestampedFrame() -> TimestampedFrame {
  mMutex.lock();
  TimestampedFrame tframe = mTimestampedFrame;
  mMutex.unlock();

  return tframe;
};

auto CameraStream::getFrame() -> cv::Mat {
  mMutex.lock();
  cv::Mat frame = mTimestampedFrame.frame;
  mMutex.unlock();

  return frame;
};
}  // namespace camera
