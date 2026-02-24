// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraConfig.h
// Purpose: Define CameraIO Configuration Structs
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/utils/PCH.h"

namespace camera {
    /** The internal characteristics of a camera */
    using camera_intrinsics_t = struct CameraIntrinsics {
        /** Optical-center X coordinate in Pixels */
        double cx;
        /** Optical-center Y coordinate in Pixels */
        double cy;
        /** Focal X-Length in Pixels */
        double fx;
        /** Focal Y-Length in Pixels */
        double fy;
        /** Radial Distortion Coefficent 1 */
        double k1;
        /** Radial Distortion Coefficent 2 */
        double k2;
        /** Radial Distortion Coefficent 3 */
        double k3;
        /** Pixel size in Meters */
        double p1;
        /** Pixel size in Meters */
        double p2;
    };

    /** The configuration of a camera */
    using camera_config_t = struct CameraConfig {
        /** Friendly nickname for Camera */
        std::string nickname;
        /** Camera Identity */
        int device_id;
        /** API backend Camera should use */
        int api_id;
        /** Codec Camera should use */
        std::string codec;
        /** Width of Camera Capture */
        int capture_width;
        /** Height of Camera Capture */
        int capture_height;
        /** FPS of Camera Capture */
        int capture_fps;
        /** Displacement of Camera WRT to Center-of-chassis */
        frc::Transform3d transform_wrt_chassis;
        /** Internal Characterization of Camera */
        camera_intrinsics_t intrinsics_calibration;
    };
}