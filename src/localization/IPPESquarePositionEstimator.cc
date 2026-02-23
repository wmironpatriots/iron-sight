// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: MultiTagPositionEstimator.cc
// Purpose: Find robot position using multiple found apriltags
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/localization/IPPESquarePositionEstimator.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/CoordinateAxis.h>
#include <frc/geometry/CoordinateSystem.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Twist3d.h>
#include <opencv2/core/hal/interface.h>
#include <units/angle.h>
#include <units/length.h>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <optional>
#include <utility>
#include <vector>
#include "src/camera/CameraConfig.h"
#include "src/localization/AprilTagSearcher.h"
#include "src/localization/PositionEstimator.h"
#include "units/length.h"
#include "src/utils/VisionUtils.h"
namespace localization {

    IPPESquarePositionEstimator::IPPESquarePositionEstimator(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig)
                                                        : fieldLayout(std::move(fieldLayout)),
                                                        cameraWrtChassis(cameraConfig.transformWrtChassis),
                                                        cameraMatrix(utils::generateCameraMatrix(cameraConfig)),
                                                        distCoeffs(utils::generateDistCoeffs(cameraConfig)) {};

    
        auto IPPESquarePositionEstimator::estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        std::vector<pose3d_estimate_t> estimates{};

        for (const found_apriltag_t& tag : found_tags){
            auto tagPose = fieldLayout.GetTagPose(tag.tag_id);
            std::vector<cv::Point2d> singleTagImagePoints = {};
            std::vector<cv::Point3d> singleTagObjectPoints = {};

            if (tagPose != std::nullopt){

                for (int i = 0; i < 4; i++){
                    singleTagImagePoints.emplace_back(tag.cornerCoords[i]);
                }

                for (auto pose : kTagCorners) {
                    singleTagObjectPoints.emplace_back(pose.X().value(), pose.Y().value(), 0);
                }

                std::vector<cv::Mat> rvecs, tvecs;
                std::vector<double> reprojectionErrors;
                cv::solvePnPGeneric(singleTagObjectPoints, 
                                    singleTagImagePoints,
                                                cameraMatrix, 
                                                distCoeffs, 
                                                rvecs, 
                                                tvecs, 
                                                false, 
                                                cv::SOLVEPNP_IPPE_SQUARE, 
                                                cv::noArray(), 
                                                cv::noArray(), 
                                                reprojectionErrors);
                

                for (int i = 0; i < rvecs.size(); i++) {
                    auto tagToCamera = utils::ConvertOpencvRvecTvecToWpiLibTransform(rvecs[i], tvecs[i]).Inverse();
                    auto cameraPose = tagPose.value().TransformBy(tagToCamera);
                    auto robotPose = cameraPose.TransformBy(cameraWrtChassis.Inverse());
                    estimates.emplace_back(pose3d_estimate_t(found_tags, robotPose, tag.timestampSeconds, reprojectionErrors[i]));
                }

            } else {

                std::cerr << "Invalid AprilTag ID found! \n";

            }

        }
    
        return estimates;
    }

}