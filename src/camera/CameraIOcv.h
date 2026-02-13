// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraIOcv.h
// Purpose: Define base camera hardware interface
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/camera/Camera.h"

namespace camera {
    /** CameraIO implementation for openCV cam */
    class CameraIOcv : CameraIO {
        public:
            CameraIOcv(const CameraConfig& config);
            auto getTimestampedFrame() -> TimestampedFrame override;
        private:
            CameraConfig mConfig;
            cv::VideoCapture mCameraCapture;
    };
}
