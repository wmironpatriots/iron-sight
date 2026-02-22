// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once
#include <frc/geometry/Translation3d.h>
#include <Eigen/Core>
#include <opencv2/core/base.hpp>
#include <frc/geometry/CoordinateSystem.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include "src/camera/Camera.h"
#include "src/camera/CameraConfig.h"
#include "src/utils/PCH.h"

namespace utils {
    /** Generate Camera Matrix from config */
    auto generateCameraMatrix(const camera::CameraConfig& config) -> cv::Mat;

    /** Generate Distance Coefficents Matrix from config */
    auto generateDistCoeffs(const camera::CameraConfig& config) -> cv::Mat;
    
    /** Convert OpenCV rvec+tvec to WpiLib Transform */
    auto ConvertOpencvRvecTvecToWpiLibTransform(cv::Mat rvec, cv::Mat tvec) -> frc::Transform3d;
}