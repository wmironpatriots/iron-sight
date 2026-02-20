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
#include <opencv2/core/hal/interface.h>
#include <units/angle.h>
#include <units/length.h>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <utility>
#include <vector>
#include "src/localization/AprilTagSearcher.h"
#include "src/localization/PositionEstimator.h"
#include "units/length.h"

namespace localization {
    MultiTagPositionEstimator::MultiTagPositionEstimator(frc::AprilTagFieldLayout fieldLayout,
                                                        cv::Mat cameraMatrix,
                                                        cv::Mat distCoeffs)
                                                        : fieldLayout(std::move(fieldLayout)),
                                                        cameraMatrix(std::move(cameraMatrix)),
                                                        distCoeffs(std::move(distCoeffs)) {};

    auto MultiTagPositionEstimator::estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        if (found_tags.empty()){
            std::vector<pose3d_estimate_t> estimates{}; 
            estimates.emplace_back(pose3d_estimate_t(frc::Pose3d(), 0, 0));
            return estimates;
        }

        std::vector<cv::Point2d> imagePoints;
        std::vector<cv::Point3d> objectPoints;

        for (found_apriltag_t tag : found_tags){

            if (fieldLayout.GetTagPose(tag.tag_id).has_value()){

                for (int i = 0; i < 4; i++){
                    imagePoints.emplace_back(tag.cornerCoords[i]);
                }

                frc::Pose3d tagPose = fieldLayout.GetTagPose(tag.tag_id).value();

                for (auto pose : kTagCorners) {
                    pose.TransformBy(frc::Transform3d(tagPose.Translation(), tagPose.Rotation()));
                    objectPoints.emplace_back(pose.X().value(), pose.Y().value(), pose.Z().value());
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
        
        units::length::meter_t x{tvec.at<double>(0, 0)};
        units::length::meter_t y{tvec.at<double>(1, 0)};
        units::length::meter_t z{tvec.at<double>(2, 0)};
        auto translation = frc::Translation3d(x, y, z);

        Eigen::Vector3d vec(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
        auto rotation = frc::Rotation3d(vec, units::angle::radian_t{cv::norm(rvec)});

        auto cvFieldToCamm = frc::Transform3d(translation, rotation).Inverse();
        auto wpilibFieldToCam = frc::CoordinateSystem::Convert(cvFieldToCamm, frc::CoordinateSystem::EDN(), frc::CoordinateSystem::NWU());

        std::vector<pose3d_estimate_t> estimates{}; 
        auto pose = frc::Pose3d();

        estimates.emplace_back(pose3d_estimate_t(pose.TransformBy(wpilibFieldToCam), timestamp, 0));

        return estimates;
    }
}