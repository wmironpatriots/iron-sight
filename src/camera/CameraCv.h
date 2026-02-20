// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraCv.h
// Purpose: Define base camera hardware interface
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/camera/Camera.h"
#include "src/camera/CameraConfig.h"

namespace camera {
    /** CameraIO implementation for openCV cam */
    class CameraCv : public Camera {
        public:
            CameraCv(const CameraConfig& config);            
            auto getFrame() -> cv::Mat override;            
            auto getTimestampedFrame() -> TimestampedFrame override;
        private:
            CameraConfig mConfig;
            cv::VideoCapture mCameraCapture;
    };
}
