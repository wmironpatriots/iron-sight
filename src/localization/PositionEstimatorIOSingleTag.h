// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimatorIOSingleTag.h
// Purpose: Define PositionEstimator with a IPPE Square PNP estimation backend
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/localization/PositionEstimatorIO.h"
#include "src/camera/CameraConfig.h"

namespace localization {
    /** A PositionEstimatorIO extension that solves based on single tags */
    class PositionEstimatorIOSingleTag : PositionEstimatorIO {
        public:
            PositionEstimatorIOSingleTag(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig);
            auto Estimate3dPoseFromFoundTags(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> override;
        private:
            frc::AprilTagFieldLayout field_layout_;
            frc::Transform3d camera_wrt_chassis_;
            cv::Mat camera_matrix_;
            cv::Mat dist_coeffs_;
    };
}
