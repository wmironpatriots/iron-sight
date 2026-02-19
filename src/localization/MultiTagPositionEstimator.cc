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
        //copied code, i have no idea how ts works
        cv::Mat Rcv;
        cv::Rodrigues(rvec, Rcv);
        cv::Mat R64;
        Rcv.convertTo(R64, CV_64F);
        Eigen::Matrix3d R;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
            R(r, c) = R64.at<double>(r, c);

        Eigen::Vector3d t_cv;
        t_cv.x() = static_cast<double>(tvec.at<double>(0));
        t_cv.y() = static_cast<double>(tvec.at<double>(1));
        t_cv.z() = static_cast<double>(tvec.at<double>(2));
        Eigen::Matrix3d R_cam_obj = R.transpose();
        Eigen::Vector3d t_cam_obj = -R.transpose() * t_cv;
        Eigen::Matrix3d P;
        P <<  0,  0,  1,
             -1,  0,  0,
              0, -1,  0;

        Eigen::Matrix3d R_wp = P * R_cam_obj * P.transpose();
        Eigen::Vector3d t_wp = P * t_cam_obj;
        Eigen::Quaternion<double> q(R_wp);
        frc::Rotation3d rot{frc::Quaternion{q.w(), q.x(), q.y(), q.z()}};
        frc::Translation3d trans{units::meter_t{t_wp.x()},
                           units::meter_t{t_wp.y()},
                           units::meter_t{t_wp.z()}};

        return frc::Pose3d{trans, rot};
    }
    auto MultiTagPositionEstimator::estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> {
        std::vector<cv::Point2d> imagePoints;
        for (found_apriltag_t tag : found_tags){
            for (int i = 0; i < 4; i++){
                imagePoints.emplace_back(tag.cornerCoords[i]);
            }
        }
        std::vector<cv::Point3d> objectPoints;
        double tag_size = 0.1651;
        double half = tag_size/2;
        for (found_apriltag_t tag : found_tags){
            // this assumes that the tag exists in the field, will crash if it doesnt. cry about it. loop above also needs to be fixed, as the two need to match 1:1
            frc::Pose3d tagPose = fieldLayout.GetTagPose(tag.tag_id).value();
            double x = tagPose.X().value();
            double y = tagPose.Y().value();
            double z = tagPose.Z().value();
            objectPoints.emplace_back(x-half, y+half, z);
            objectPoints.emplace_back(x+half, y+half, z);
            objectPoints.emplace_back(x-half, y-half, z);
            objectPoints.emplace_back(x+half, y-half, z);
        }
        cv::Mat rvec, tvec;
        //TODO there are other methods than SQPNP, try them later
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
        
        double timestamp = 0.0;
        if (!found_tags.empty()){
            timestamp = found_tags[0].timestampSeconds;
        }
        std::vector<pose3d_estimate_t> estimates{}; 
        estimates.emplace_back(pose3d_estimate_t(OpencvRvecTvec2WpilibPose3d(rvec, tvec), timestamp, 0));
        return estimates;
    }
}