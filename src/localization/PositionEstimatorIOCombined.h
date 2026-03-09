// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimatorIOCombined.h
// Purpose: Define the PositionEstimatorIOCombined class and the structs it uses
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once
#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include <frc/geometry/Rotation3d.h>
#include "src/localization/PositionEstimatorIO.h"
#include "src/localization/TagSearcherIO.h"

namespace localization {
    /** A PositionEstimator that finds the optimal position estimator to use */
    class PositionEstimatorIOCombined : public PositionEstimatorIO {
        public:
            PositionEstimatorIOCombined(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig);
            auto Estimate3dPoseFromFoundTags(const found_apriltags_in_frame_t& found_tags) -> std::vector<pose3d_estimate_t> override;
        private:
            frc::AprilTagFieldLayout field_layout_;
            frc::Transform3d camera_wrt_chassis_;
            cv::Mat camera_matrix_;
            cv::Mat dist_coeffs_;
            localization::PositionEstimatorIOMultiTag multitag_position_estimator_;
            localization::PositionEstimatorIOSingleTag singletag_position_estimator_;

    };
}
