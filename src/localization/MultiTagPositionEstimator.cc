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
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>
#include <Eigen/Core>
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

    auto MultiTagPositionEstimator::OpencvRvecTvec2WpilibPose3d(cv::Mat& rvec, cv::Mat& tvec) -> frc::Pose3d {
        units::length::meter_t x{tvec.at<double>(0, 0)};
        units::length::meter_t y{tvec.at<double>(1, 0)};
        units::length::meter_t z{tvec.at<double>(2, 0)};
        auto translation = frc::Translation3d(x, y, z);

        Eigen::Vector3d vec(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
        auto rotation = frc::Rotation3d(vec, units::angle::radian_t{cv::norm(rvec)});

        auto cvPose = frc::Pose3d(translation, rotation);

        return frc::CoordinateSystem::Convert(cvPose, frc::CoordinateSystem::EDN(), frc::CoordinateSystem::NWU());
    }

    auto MultiTagPositionEstimator::estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        if (found_tags.empty()){
            std::vector<pose3d_estimate_t> estimates{}; 
            estimates.emplace_back(pose3d_estimate_t(frc::Pose3d(), 0, 0));
            return estimates;
        }
        std::vector<cv::Point2d> imagePoints;
        std::vector<cv::Point3d> objectPoints;
        double tag_size = 0.1651;
        double half = tag_size/2;
        for (found_apriltag_t tag : found_tags){
            if (!fieldLayout.GetTagPose(tag.tag_id).has_value()){
                std::cerr << "Invalid AprilTag ID found! \n";
                continue;
            }
            for (int i = 0; i < 4; i++){
                imagePoints.emplace_back(tag.cornerCoords[i]);
            }
            frc::Pose3d tagPose = fieldLayout.GetTagPose(tag.tag_id).value();
            //std::cerr << "debug \n";
            frc::Pose3d corner0 = tagPose.TransformBy(frc::Transform3d{frc::Translation3d{-units::meter_t{half}, units::meter_t{half}, 0_m}, frc::Rotation3d{}});
            frc::Pose3d corner1 = tagPose.TransformBy(frc::Transform3d{frc::Translation3d{units::meter_t{half}, units::meter_t{half}, 0_m}, frc::Rotation3d{}});
            frc::Pose3d corner2 = tagPose.TransformBy(frc::Transform3d{frc::Translation3d{units::meter_t{half}, -units::meter_t{half}, 0_m}, frc::Rotation3d{}});
            frc::Pose3d corner3 = tagPose.TransformBy(frc::Transform3d{frc::Translation3d{-units::meter_t{half}, -units::meter_t{half}, 0_m}, frc::Rotation3d{}});

            objectPoints.emplace_back(corner0.X().value(), corner0.Y().value(), corner0.Z().value());
            objectPoints.emplace_back(corner1.X().value(), corner1.Y().value(), corner1.Z().value());
            objectPoints.emplace_back(corner2.X().value(), corner2.Y().value(), corner2.Z().value());
            objectPoints.emplace_back(corner3.X().value(), corner3.Y().value(), corner3.Z().value());
        }

        cv::Mat rvec, tvec;
        //TODO there are other methods than SQPNP, try them later
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
        
        double timestamp = 0.0;
        if (!found_tags.empty()){
            timestamp = found_tags[0].timestampSeconds;
        }
        std::vector<pose3d_estimate_t> estimates{}; 
        //std::cerr << rvec << tvec;
        estimates.emplace_back(pose3d_estimate_t(OpencvRvecTvec2WpilibPose3d(rvec, tvec), timestamp, 0));
        return estimates;
    }
}