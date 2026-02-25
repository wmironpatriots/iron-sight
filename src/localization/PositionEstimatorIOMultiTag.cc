// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: PositionEstimatorIOMultiTag.h
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/utils/CalibrationUtils.h"
#include "src/utils/GeometryUtils.h"

namespace localization {
    PositionEstimatorIOMultiTag::PositionEstimatorIOMultiTag(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig) : 
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

    
    auto PositionEstimatorIOMultiTag::Estimate3dPoseFromFoundTags(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        std::vector<pose3d_estimate_t> estimates{};

        std::vector<cv::Point2d> imagePoints;
        std::vector<cv::Point3d> objectPoints;

        for (found_apriltag_t tag : found_tags){

            auto tagPose = field_layout_.GetTagPose(tag.tag_id);

            if (tagPose != std::nullopt){

                auto cvPose = utils::WpilibCoordSysToOpenCvCoordSys(tagPose.value());

                for (int i = 0; i < 4; i++){
                    imagePoints.emplace_back(tag.corner_coords[i]);
                }

                for (auto pose : kTagCorners) {
                    auto cornerTransform = frc::Transform3d(cvPose.Translation(), cvPose.Rotation());

                    auto cornerPose = pose.TransformBy(cornerTransform);
                    objectPoints.emplace_back(cornerPose.X().value(), cornerPose.Y().value(), cornerPose.Z().value());
                }

            } else {

                std::cerr << "Invalid AprilTag ID found! \n";

            }

        }

        if (objectPoints.empty() && imagePoints.empty()){
            return {};
        }

        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, 
                            imagePoints, 
                            camera_matrix_, 
                            dist_coeffs_, 
                            rvec, 
                            tvec, 
                            false, 
                            cv::SOLVEPNP_SQPNP);

        auto cameraPose = frc::Pose3d().TransformBy(utils::OpenCvTransformToWpilibTransform(rvec, tvec));
        auto robotPose = cameraPose.TransformBy(camera_wrt_chassis_.Inverse());
        estimates.emplace_back(pose3d_estimate_t(found_tags, robotPose, found_tags[0].timestamp_seconds, 0));

        return estimates;
    }
}