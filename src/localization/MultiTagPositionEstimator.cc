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
#include <frc/geometry/Pose3d.h>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <utility>
#include <vector>
#include "src/localization/AprilTagSearcher.h"
#include "src/localization/PositionEstimator.h"

namespace localization {
    MultiTagPositionEstimator::MultiTagPositionEstimator(frc::AprilTagFieldLayout fieldLayout,
                                                        cv::Mat cameraMatrix,
                                                        cv::Mat distCoeffs)
                                                        : fieldLayout(std::move(fieldLayout)),
                                                        cameraMatrix(std::move(cameraMatrix)),
                                                        distCoeffs(std::move(distCoeffs)) {};
    auto MultiTagPositionEstimator::OpencvRvecTvec2WpilibPose3d(cv::Mat& rvec, cv::Mat& tvec) -> frc::Pose3d {
        cv::Mat rotation_cv;
        cv::Rodrigues(rvec, rotation_cv);

        cv::Mat rotation64;
        rotation_cv.convertTo(rotation64, CV_64F);

        cv::Mat translation64;
        tvec.convertTo(translation64, CV_64F);

        Eigen::Matrix3d rotation_object_to_camera_mixed;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                rotation_object_to_camera_mixed(r, c) = rotation64.at<double>(r, c);
            }
        }

        Eigen::Vector3d translation_object_to_camera;
        translation_object_to_camera.x() = translation64.at<double>(0, 0);
        translation_object_to_camera.y() = translation64.at<double>(1, 0);
        translation_object_to_camera.z() = translation64.at<double>(2, 0);

        // OpenCV solvePnP returns X_cam(cv) = R * X_obj(wpi) + t.
        // Invert to get camera origin in object/frame coordinates.
        Eigen::Matrix3d rotation_camera_to_object_mixed = rotation_object_to_camera_mixed.transpose();
        Eigen::Vector3d translation_camera_in_object_wpi =
            -rotation_camera_to_object_mixed * translation_object_to_camera;

        // OpenCV camera axes -> WPILib camera axes:
        // cv: +x right, +y down, +z forward
        // wpi: +x forward, +y left, +z up
        Eigen::Matrix3d cv_to_wpi;
        cv_to_wpi << 0, 0, 1,
                    -1, 0, 0,
                     0, -1, 0;

        // Only one basis conversion is needed here. Left-multiplying again
        // would apply the axis remap twice and swap/negate incorrectly.
        Eigen::Matrix3d rotation_camera_to_object_wpi = rotation_camera_to_object_mixed * cv_to_wpi.transpose();

        Eigen::Quaterniond q(rotation_camera_to_object_wpi);
        frc::Rotation3d rot{frc::Quaternion{q.w(), q.x(), q.y(), q.z()}};
        frc::Translation3d trans{
            units::meter_t{translation_camera_in_object_wpi.x()},
            units::meter_t{translation_camera_in_object_wpi.y()},
            units::meter_t{translation_camera_in_object_wpi.z()}
        };

        return frc::Pose3d{trans, rot};
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
            double x = tagPose.X().value();
            double y = tagPose.Y().value();
            double z = tagPose.Z().value();
            //std::cerr << x << " " << y << " " << z << "\n";
            objectPoints.emplace_back(x-half, y+half, z);
            objectPoints.emplace_back(x+half, y+half, z);
            objectPoints.emplace_back(x+half, y-half, z);
            objectPoints.emplace_back(x-half, y-half, z);
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