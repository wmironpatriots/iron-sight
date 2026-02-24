// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraStream.h
// Purpose: Define CameraStream class
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/camera/Camera.h"

namespace camera {
    /** A continous stream of timestamped frames from a CameraIO */
    class CameraStream {
        public:
            CameraStream(std::unique_ptr<CameraIO> IO);
            /** Return latest frame from stream */
            auto getTimestampedFrame() -> TimestampedFrame;
        private:
            std::unique_ptr<CameraIO> io_;
            TimestampedFrame timestamped_frame_;
            std::thread thread_;
            std::mutex mutex_;
    };
}
