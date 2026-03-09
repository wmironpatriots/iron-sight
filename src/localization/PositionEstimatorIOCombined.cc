// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: PositionEstimatorIOCombined.h
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/localization/PositionEstimatorIOCombined.h"
#include <frc/geometry/Rotation3d.h>

#include <utility>
#include "src/localization/PositionEstimatorIO.h"
#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include "src/localization/TagSearcherIO.h"
#include "src/utils/CalibrationUtils.h"

namespace localization {
    PositionEstimatorIOCombined::PositionEstimatorIOCombined(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig) : 
        field_layout_(std::move(fieldLayout)), 
        camera_wrt_chassis_(cameraConfig.transform_wrt_chassis),
        camera_matrix_(utils::CameraMatrixFromIntrinsics(
            cameraConfig.intrinsics_calibration.fx,
            cameraConfig.intrinsics_calibration.fy, 
            cameraConfig.intrinsics_calibration.cx, 
            cameraConfig.intrinsics_calibration.cy)),
        dist_coeffs_(utils::DistortionCoefficentsFromIntrinsics(
            cameraConfig.intrinsics_calibration.p1, 
            cameraConfig.intrinsics_calibration.p2, 
            cameraConfig.intrinsics_calibration.k1, 
            cameraConfig.intrinsics_calibration.k2, 
            cameraConfig.intrinsics_calibration.k3)),
        multitag_position_estimator_(localization::PositionEstimatorIOMultiTag(field_layout_, cameraConfig)),
        singletag_position_estimator_(localization::PositionEstimatorIOSingleTag(field_layout_, cameraConfig)) {} 
    
    auto PositionEstimatorIOCombined::Estimate3dPoseFromFoundTags(const found_apriltags_in_frame_t& found_tags) -> std::vector<pose3d_estimate_t> {
        if (found_tags.found_tags.size() == 0){
            return {};
        }

        else if (found_tags.found_tags.size() == 1) {
            return singletag_position_estimator_.Estimate3dPoseFromFoundTags(found_tags);
        }

        return multitag_position_estimator_.Estimate3dPoseFromFoundTags(found_tags);
    }
}