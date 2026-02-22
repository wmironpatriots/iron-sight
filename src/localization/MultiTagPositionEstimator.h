// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: MultiTagPositionEstimator.h
// Purpose: Define PositionEstimator with multitag estimation backend
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include <frc/geometry/Transform3d.h>
#include <opencv2/core/mat.hpp>
#include "src/camera/CameraConfig.h"
#include "src/localization/PositionEstimator.h"
#include "apriltag/frc/apriltag/AprilTagFieldLayout.h"

namespace localization {
    
    /** Represents a PositionEstimator that solves based on many found tags */
    class MultiTagPositionEstimator : IPositionEstimator {
        public:
            MultiTagPositionEstimator(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig);
            auto estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> override;
        private:
            frc::AprilTagFieldLayout fieldLayout;
            frc::Transform3d cameraWrtChassis;
            cv::Mat cameraMatrix;
            cv::Mat distCoeffs;
    };
}
