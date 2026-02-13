// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraStream.h
// Purpose: Define camera streams
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/camera/Camera.h"

namespace camera {
    /** Represents a continious stream of timestamped frames from camera */
    class CameraStream {
        public:
            CameraStream(std::unique_ptr<CameraIO> IO);
            /** Return lastest timestamped frame from stream */
            auto getTimestampedFrame() -> TimestampedFrame;
            /** Return latest raw frame */
            auto getFrame() -> cv::Mat;
        private:
            std::unique_ptr<CameraIO> mIO;
            TimestampedFrame mTimestampedFrame;
            std::thread mThread;
            std::mutex mMutex;
    };
}
