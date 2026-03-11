// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: PositionEstimatorIOSingleTag.h
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include "src/localization/PositionEstimatorIO.h"
#include "src/localization/TagSearcherIO.h"
#include "src/utils/CalibrationUtils.h"
#include "src/utils/GeometryUtils.h"

namespace localization {
    PositionEstimatorIOSingleTag::PositionEstimatorIOSingleTag(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig) :
        field_layout_(std::move(fieldLayout)), 
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

    
    auto PositionEstimatorIOSingleTag::Estimate3dPoseFromFoundTags(const found_apriltags_in_frame_t& found_tags) -> std::vector<pose3d_estimate_t> {
        std::vector<pose3d_estimate_t> estimates{};

        for (const found_apriltag_t& tag : found_tags.found_tags){
            auto tagPose = field_layout_.GetTagPose(tag.tag_id);
            std::vector<cv::Point2d> singleTagImagePoints;
            std::vector<cv::Point3d> singleTagObjectPoints;
            singleTagImagePoints.reserve(4);
            singleTagObjectPoints.reserve(4);

            if (tagPose != std::nullopt){

                for (int i = 0; i < 4; i++){
                    singleTagImagePoints.emplace_back(tag.corner_coords[i]);
                }

                for (int i = 0; i < 4; i++) {
                    singleTagObjectPoints.emplace_back(kTagCorners[i][0], kTagCorners[i][1], 0);
                    //singleTagObjectPoints.emplace_back(corner.X().value(), corner.Y().value(), corner.Z().value());
                }

                std::vector<cv::Mat> rvecs;
                std::vector<cv::Mat> tvecs;
                cv::Mat reprojectionErrors;
                cv::solvePnPGeneric(singleTagObjectPoints, 
                                    singleTagImagePoints, 
                                    camera_matrix_, 
                                    dist_coeffs_, 
                                    rvecs, 
                                    tvecs, 
                                    false, 
                                    cv::SOLVEPNP_IPPE_SQUARE, 
                                    cv::noArray(), 
                                    cv::noArray(), 
                                    reprojectionErrors);
                
                auto cameraWrtTag = utils::OpenCvTransformToWpilibTransform(rvecs[0], tvecs[0]);
                auto cameraPose = tagPose.value().TransformBy(cameraWrtTag);
                estimates.emplace_back(pose3d_estimate_t(cameraPose, found_tags.timestamp_seconds, reprojectionErrors.at<double>(0), 1));

            } else {

                std::cerr << "Invalid AprilTag ID found! \n";

            }

        }
    
        return estimates;
    }

}