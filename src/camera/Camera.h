// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: Camera.h
// Purpose: Define the CameraIO class and the structs it uses
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/camera/CameraConfig.h"
#include "src/utils/PCH.h"

namespace camera {
    /** Camera frame recorded at a specified timestamp */
    struct TimestampedFrame {
        /** Image data of the frame */
        cv::Mat frame;
        /** Match timestamp frame was recorded in */
        units::second_t timestamp;
    };

    /** Hardware Interface for interacting with a Camera */
    class CameraIO {
        public:
            virtual ~CameraIO() = default;
            /** Return camera configuration */
            virtual auto GetConfig() -> camera_config_t;
            /** Return the latest timestamped frame recorded by camera */
            virtual auto GetTimestampedFrame() -> TimestampedFrame = 0;
            /** Attempt to reinitialize camera */
            virtual auto Restart() -> void;
    };
}