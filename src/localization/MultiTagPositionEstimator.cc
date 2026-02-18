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
#include "src/localization/AprilTagSearcher.h"

namespace localization {
    MultiTagPositionEstimator::MultiTagPositionEstimator(const frc::AprilTagFieldLayout& fieldLayout, 
                                                        const cv::Mat& cameraMatrix, 
                                                        const cv::Mat& distCoeffs) 
                                                        : fieldLayout(fieldLayout),
                                                        cameraMatrix(cameraMatrix),
                                                        distCoeffs(distCoeffs) {};
    auto MultiTagPositionEstimator::OpencvTransformation2WpilibPose3d(cv::Mat& rvec, cv::Mat& tvec) -> frc::Pose3d {
        
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
            objectPoints.emplace_back(cv::Point3d(x-half, y+half, z));
            objectPoints.emplace_back(cv::Point3d(x+half, y+half, z));
            objectPoints.emplace_back(cv::Point3d(x-half, y-half, z));
            objectPoints.emplace_back(cv::Point3d(x-half, y-half, z));
        }
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        
    }
}