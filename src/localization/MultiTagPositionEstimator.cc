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

    auto ConvertOpencvRvecTvecToWpiLibTransform(cv::Mat rvec, cv::Mat tvec) -> frc::Transform3d {
        Eigen::Vector3d T(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        auto translation = frc::Translation3d(T);

        Eigen::Vector3d R(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        auto rotation = frc::Rotation3d(R, units::angle::radian_t{R.norm()});

        auto cvFieldToCamera = frc::Transform3d(translation, rotation).Inverse();
        auto wpilibFieldToCamera = frc::CoordinateSystem::Convert(cvFieldToCamera, frc::CoordinateSystem::EDN(), frc::CoordinateSystem::NWU());
        return wpilibFieldToCamera;
    }
    
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
        std::vector<pose3d_estimate_t> estimates{};

        std::vector<cv::Point2d> imagePoints;
        std::vector<cv::Point3d> objectPoints;

        for (found_apriltag_t tag : found_tags){
            std::vector<cv::Point2d> singleTagImagePoints{};
            std::vector<cv::Point3d> singleTagObjectPoints{};
            if (fieldLayout.GetTagPose(tag.tag_id) != std::nullopt){

                for (int i = 0; i < 4; i++){
                    singleTagImagePoints.emplace_back(tag.cornerCoords[i]);
                    imagePoints.emplace_back(tag.cornerCoords[i]);
                }

                auto cvTagPose = frc::CoordinateSystem::Convert(fieldLayout.GetTagPose(tag.tag_id).value(), frc::CoordinateSystem::NWU(), frc::CoordinateSystem::EDN());

                for (auto pose : kTagCorners) {
                    auto cornerTransform = frc::Transform3d(cvTagPose.Translation(), cvTagPose.Rotation());

                    auto cornerPose = pose.TransformBy(cornerTransform);
                    singleTagObjectPoints.emplace_back(cornerPose.X().value(), cornerPose.Y().value(), cornerPose.Z().value());
                    objectPoints.emplace_back(cornerPose.X().value(), cornerPose.Y().value(), cornerPose.Z().value());
                }
                cv::Mat singleTagRvec, singleTagTvec;
                cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, singleTagRvec, singleTagTvec, false, cv::SOLVEPNP_IPPE_SQUARE);
                
                auto pose = frc::Pose3d().TransformBy(ConvertOpencvRvecTvecToWpiLibTransform(singleTagRvec, singleTagTvec));

                estimates.emplace_back(pose3d_estimate_t(pose, tag.timestampSeconds, 1));
            } else {

                std::cerr << "Invalid AprilTag ID found! \n";

            }

        }
        if (objectPoints.empty() && imagePoints.empty()){
            return {};
        }
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
        auto pose = frc::Pose3d().TransformBy(ConvertOpencvRvecTvecToWpiLibTransform(rvec, rvec));
        double timestamp = 0.0;
        timestamp = found_tags[0].timestampSeconds;
        estimates.emplace_back(pose3d_estimate_t(pose, timestamp, 1));

        return estimates;
    }
}