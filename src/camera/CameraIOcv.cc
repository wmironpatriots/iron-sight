// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraIOcv.cc
// Purpose: Define base camera hardware interface
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/camera/CameraIOcv.h"
#include <frc/Timer.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include "src/camera/Camera.h"

namespace camera {
    CameraIOcv::CameraIOcv(const CameraConfig& config) {
        mConfig = config;
        mCameraCapture = cv::VideoCapture(config.deviceId, config.apiId);

        // Check if stream connected to camera
        if (!mCameraCapture.isOpened()) {
            printf("Error ~ Failed to open camera\n");
            throw;
        }
    }

    auto CameraIOcv::getTimestampedFrame() -> TimestampedFrame {
        TimestampedFrame tframe;
        mCameraCapture.read(tframe.frame);
        tframe.timestamp = frc::Timer::GetFPGATimestamp();

        return tframe;
    }
}
