// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraConfig.h
// Purpose: Define structs for characterizing cameras
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include <frc/geometry/Rotation3d.h>
#include <opencv2/videoio.hpp>
#include "src/utils/PCH.h"

namespace camera {
    using camera_extrinsics_t = struct CameraExtrinsicsCalibration {
        frc::Translation3d positionalDisplacement;
        frc::Rotation3d rotationalDisplacement;
    };

    using camera_intrinsics_t = struct CameraIntrinsics {
        double cx;
        double cy;
        double fx;
        double fy;
        double k1;
        double k2;
        double k3;
        double p1;
        double p2;
    };

    /** Represents a camera  */
    using camera_config_t = struct CameraConfig {
        /** camera id */
        int deviceId;
        /** API backend to use */
        int apiId;
        /** Codec to use */
        std::string codec;
        /** Height of capture*/
        int captureHeight;
        /** Width of capture */
        int captureWidth;
        /** FPS of capture */
        int captureFPS;
        camera_extrinsics_t extrinsicsCalibration;
        camera_intrinsics_t intrinsicsCalibration;
    };

    inline const camera_config_t kMultiTagDemoCam = camera_config_t{
        2,
        cv::CAP_V4L2,
        "MJPG",
        800,
        600,
        100,
        camera_extrinsics_t{
            frc::Translation3d(),
            frc::Rotation3d()
        },
        camera_intrinsics_t{
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        }
    };
}