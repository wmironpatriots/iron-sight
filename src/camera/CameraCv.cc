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
        mCameraCapture.set(cv::CAP_PROP_FRAME_WIDTH, config.captureWidth);
        mCameraCapture.set(cv::CAP_PROP_FRAME_HEIGHT, config.captureHeight); 
        mCameraCapture.set(cv::CAP_PROP_FPS, config.captureFPS);
        std::string codec = config.codec;
        mCameraCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]));
        // Check if stream connected to camera
        if (!mCameraCapture.isOpened()) {
            printf("Error ~ Failed to open camera\n");
            throw;
        }
    }

    auto CameraCv::getFrame() -> cv::Mat {
        cv::Mat frame;
        mCameraCapture.read(frame);
        return frame;
    }

    auto CameraCv::getTimestampedFrame() -> TimestampedFrame {
        TimestampedFrame tframe;
        mCameraCapture.read(tframe.frame);
        tframe.timestamp = frc::Timer::GetFPGATimestamp();

        return tframe;
    }
}
