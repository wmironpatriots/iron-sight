// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: MultiTagPositionEstimator.cc
// Purpose: Find robot position using multiple found apriltags
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/localization/MultiTagPositionEstimator.h"
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

    MultiTagPositionEstimator::MultiTagPositionEstimator(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig)
                                                        : fieldLayout(std::move(fieldLayout)),
                                                        cameraWrtChassis(cameraConfig.transformWrtChassis),
                                                        cameraMatrix(utils::generateCameraMatrix(cameraConfig)),
                                                        distCoeffs(utils::generateDistCoeffs(cameraConfig)) {};

    
        auto MultiTagPositionEstimator::estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        std::vector<pose3d_estimate_t> estimates{};

        std::vector<cv::Point2d> imagePoints;
        std::vector<cv::Point3d> objectPoints;

        for (found_apriltag_t tag : found_tags){

            auto tagPose = fieldLayout.GetTagPose(tag.tag_id);

            if (tagPose != std::nullopt){

                for (int i = 0; i < 4; i++){
                    imagePoints.emplace_back(tag.cornerCoords[i]);
                }

                auto cvTagPose = frc::CoordinateSystem::Convert(fieldLayout.GetTagPose(tag.tag_id).value(), frc::CoordinateSystem::NWU(), frc::CoordinateSystem::EDN());

                for (auto pose : kTagCorners) {
                    auto cornerTransform = frc::Transform3d(cvTagPose.Translation(), cvTagPose.Rotation());
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

        std::vector<cv::Mat> rvecs, tvecs;
        std::vector<double> reprojectionErrors;
        cv::solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray());
        
        for (int i = 0; i < 2; i++) {
                auto cameraPose = frc::Pose3d().TransformBy(utils::ConvertOpencvRvecTvecToWpiLibTransform(rvecs[i], tvecs[i]));
                auto robotPose = cameraPose.TransformBy(cameraWrtChassis.Inverse());
                estimates.emplace_back(pose3d_estimate_t(robotPose, found_tags[0].timestampSeconds, reprojectionErrors[i]));
        }

        return estimates;
    }
}