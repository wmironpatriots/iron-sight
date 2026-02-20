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
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>
#include "src/utils/PCH.h"

namespace camera {
    /** Represent the internal characteristics of a camera */
    using camera_intrinsics_t = struct CameraIntrinsics {
        /** Optical Center X coord in Pixels */
        double cx;
        /** Optical Center Y coord in Pixels */
        double cy;
        /** Focal X Length in Pixels */
        double fx;
        /** Focal Y Length in Pixels */
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
        /** Represents the displacement of camera WRT chassis center */
        frc::Transform3d transformWrtChassis;
        /** The internal characteristics of camera */
        camera_intrinsics_t intrinsicsCalibration;
    };
}