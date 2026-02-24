// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: GeometryUtils.h
// Purpose: Define utility functions for interacting with geometry objects from vendors
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include "src/utils/PCH.h"

namespace utils {
    /** Convert from OpenCV transform vectors (rvec, tvec) to WPIlib Transform3d */
    auto OpenCvTransformToWpilibTransform(cv::Mat rvec, cv::Mat tvec) -> frc::Transform3d;

    /** Convert Pose3d from WPIlib coordinate system to OpenCV coordinate system */
    auto WpilibCoordSysToOpenCvCoordSys(frc::Pose3d pose) -> frc::Pose3d;

    /** Convert Transform3d from WPIlib coordinate system to OpenCV coordinate system */
    auto WpilibCoordSysToOpenCvCoordSys(frc::Transform3d transform) -> frc::Transform3d;

    /** Convert Pose3d from OpenCV coordinate system to WPIlib coordinate system */
    auto OpenCvCoordSysToWpilibCoordSys(frc::Pose3d pose) -> frc::Pose3d;

    /** Convert Pose3d from OpenCV coordinate system to WPIlib coordinate system */
    auto OpenCvCoordSysToWpilibCoordSys(frc::Transform3d transform) -> frc::Transform3d;
}