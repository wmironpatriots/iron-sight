// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: PositionEstimatorIOSingleTag.h
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include "src/utils/CalibrationUtils.h"
#include "src/utils/GeometryUtils.h"

namespace localization {
    PositionEstimatorIOSingleTag::PositionEstimatorIOSingleTag(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig) :
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
            cameraConfig.intrinsics_calibration.k3)) {};

    
    auto PositionEstimatorIOSingleTag::Estimate3dPoseFromFoundTags(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        std::vector<pose3d_estimate_t> estimates{};

        for (const found_apriltag_t& tag : found_tags){
            auto tagPose = field_layout_.GetTagPose(tag.tag_id);
            std::vector<cv::Point2d> singleTagImagePoints = {};
            std::vector<cv::Point3d> singleTagObjectPoints = {};

            if (tagPose != std::nullopt){

                for (int i = 0; i < 4; i++){
                    singleTagImagePoints.emplace_back(tag.corner_coords[i]);
                }

                for (const auto& corner : kTagCorners) {
                    singleTagObjectPoints.emplace_back(corner.X().value(), corner.Y().value(), corner.Z().value());
                }

                cv::Mat rvec, tvec;
                cv::solvePnP(singleTagObjectPoints, 
                                    singleTagImagePoints,
                                                camera_matrix_, 
                                                dist_coeffs_, 
                                                rvec, 
                                                tvec, 
                                                false, 
                                                cv::SOLVEPNP_IPPE_SQUARE);
                
                auto cameraWrtTag = utils::OpenCvTransformToWpilibTransform(rvec, tvec);
                auto cameraPose = tagPose.value().TransformBy(cameraWrtTag);
                auto robotPose = cameraPose.TransformBy(camera_wrt_chassis_.Inverse());
                estimates.emplace_back(pose3d_estimate_t(found_tags, robotPose, tag.timestamp_seconds, 0));

            } else {

                std::cerr << "Invalid AprilTag ID found! \n";

            }

        }
    
        return estimates;
    }

}