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
#include <iterator>
#include <numeric>
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

namespace localization {
    auto generateCameraMatrix(const camera::CameraConfig& config) -> cv::Mat {
        auto intrinsics = config.intrinsicsCalibration;
        cv::Mat matrix = (cv::Mat_<double>(3, 3) <<
            intrinsics.fx, 0,             intrinsics.cx,
            0,             intrinsics.fy, intrinsics.cy,
            0,             0,             1);

        return matrix;
    }

    auto generateDistCoeffs(const camera::CameraConfig& config) -> cv::Mat {
        auto intrinsics = config.intrinsicsCalibration;
        cv::Mat matrix = (cv::Mat_<double>(1, 5) <<
            intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2, intrinsics.k3);

        return matrix;
    }

    MultiTagPositionEstimator::MultiTagPositionEstimator(frc::AprilTagFieldLayout fieldLayout, const camera::CameraConfig& cameraConfig)
                                                        : fieldLayout(std::move(fieldLayout)),
                                                        cameraWrtChassis(cameraConfig.transformWrtChassis),
                                                        cameraMatrix(generateCameraMatrix(cameraConfig)),
                                                        distCoeffs(generateDistCoeffs(cameraConfig)) {};

    auto MultiTagPositionEstimator::estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        if (found_tags.empty()){

            std::vector<pose3d_estimate_t> estimates{}; 
            estimates.emplace_back(pose3d_estimate_t(frc::Pose3d(), 0, 0));
            return estimates;

        }

        std::vector<cv::Point2d> imagePoints;
        std::vector<cv::Point3d> objectPoints;

        for (found_apriltag_t tag : found_tags){

            if (fieldLayout.GetTagPose(tag.tag_id) != std::nullopt){

                for (int i = 0; i < 4; i++){
                    imagePoints.emplace_back(tag.cornerCoords[i]);
                }

                frc::Pose3d tagPose = fieldLayout.GetTagPose(tag.tag_id).value();

                for (auto pose : kTagCorners) {
                    auto wpilibTransform = frc::Transform3d(tagPose.Translation(), tagPose.Rotation());
                    auto cvTransform = frc::CoordinateSystem::Convert(wpilibTransform, frc::CoordinateSystem::NWU(), frc::CoordinateSystem::EDN());

                    pose.TransformBy(cvTransform);

                    objectPoints.emplace_back(pose.Y().value(), pose.Y().value(), pose.Z().value());
                }

            } else {

                std::cerr << "Invalid AprilTag ID found! \n";

            }

        }

        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
        
        double timestamp = 0.0;
        if (!found_tags.empty()){
            timestamp = found_tags[0].timestampSeconds;
        }

        Eigen::Vector3d T(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        auto translation = frc::Translation3d(T);

        Eigen::Vector3d R(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        auto rotation = frc::Rotation3d(R, units::angle::radian_t{R.norm()});

        auto cvFieldToCamera = frc::Transform3d(translation, rotation);
        auto wpilibFieldToCamera = frc::CoordinateSystem::Convert(cvFieldToCamera, frc::CoordinateSystem::EDN(), frc::CoordinateSystem::NWU());

        auto pose = frc::Pose3d().TransformBy(wpilibFieldToCamera);

        std::vector<pose3d_estimate_t> estimates{}; 
        estimates.emplace_back(pose3d_estimate_t(pose, timestamp, 1));

        return estimates;
    }
}