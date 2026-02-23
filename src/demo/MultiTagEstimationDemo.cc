// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: MultiTagEstimationDemo
// Purpose: Showcase position estimation using multiple tags
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
// TODO cleanup

#include <frc/apriltag/AprilTagFields.h>
#include "src/camera/Camera.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraCv.h"
#include "src/camera/CameraStream.h"
#include "src/localization/CvAprilTagSearcher.h"
#include "src/localization/MultiTagPositionEstimator.h"
#include "src/localization/IPPESquarePositionEstimator.h"
#include "src/utils/EstimatePublisher.h"
#include "src/utils/NtUtils.h"
#include <cmath>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/version.hpp>
#include <chrono>
#include <thread>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/CoordinateSystem.h>
#include <ntcore_cpp.h>
#include <cstdlib>

namespace {
    auto PumpGuiEventsAndGetKey() -> int {
    #if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
        return cv::pollKey();
    #else
        return cv::waitKey(1);
    #endif
    }
}

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    2,
    cv::CAP_V4L2,
    "MJPG",
    800,
    600,
    120,
    frc::Transform3d(),
    camera::camera_intrinsics_t{
        619.9026951017695,
        372.2792812903024,
        539.3898006061588,
        539.0735059998198,
        -0.3241468189388152,
        0.10782527225392564,
        -0.016410664585260946,
        0.00013396458313481827,
        -0.0002794187083645791
    }
};

auto main() -> int {
    camera::CameraCv camera(kDemoCam);

    localization::CvAprilTagSearcher searcher;
    const frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);

    auto poseEstimator = localization::MultiTagPositionEstimator(fieldLayout, kDemoCam);
    auto squarePoseEstimator = localization::IPPESquarePositionEstimator(fieldLayout, kDemoCam);
    const std::string windowName = "AprilTag Detection Demo";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::namedWindow("2", cv::WINDOW_NORMAL);

    utils::StartNetworkTables();
    auto publisher = utils::EstimatePublisher("bruh");

    int count = 0;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        camera::TimestampedFrame tframe = camera.getTimestampedFrame();
        auto detections = searcher.findTags(tframe);
        auto pose = poseEstimator.estimatePosition(detections);
        auto squarepose = squarePoseEstimator.estimatePosition(detections);
        cv::Mat fieldImg = cv::imread("./resources/field.png");


        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (!pose.empty()){
            if (count >= 1){
                std::system("clear");
                std::cerr << "FPS: " << 1/elapsed.count() << "\n";
                std::cerr << pose[0].position.X().value() << "\n";
                std::cerr << pose[0].position.Y().value() << "\n"; 
                std::cerr << pose[0].position.Z().value() << "\n";
                count = 0;
                
            }
            count++;

        
            for (auto position : pose) {
                auto point = cv::Point2d((position.position.X().value() / 16.540988) * fieldImg.cols, (position.position.Y().value() / 8.069326) * fieldImg.rows);
                cv::circle(fieldImg, point, 15, cv::Scalar(0, 0, 255), -1);
                publisher.publish(position);
            }
            for (auto position : squarepose) {
                auto point = cv::Point2d((position.position.X().value() / 16.540988) * fieldImg.cols, (position.position.Y().value() / 8.069326) * fieldImg.rows);
                cv::circle(fieldImg, point, 15, cv::Scalar(255, 0, 0), -1);
            }
        }

        for (const auto& detection : detections){
            std::vector<cv::Point> corners;
            corners.reserve(detection.cornerCoords.size());
        for (const auto& corner : detection.cornerCoords) {
                corners.emplace_back(
                    static_cast<int>(std::lround(corner.x)),
                    static_cast<int>(std::lround(corner.y))
                );
            }
            std::vector<std::vector<cv::Point>> contours = {corners};
            cv::polylines(tframe.frame, contours, true, cv::Scalar(0, 255, 0), 7);
            for (auto & corner : corners) {
                cv::circle(tframe.frame, corner, 15, cv::Scalar(0, 0, 255), -1);
            }
            cv::Point2d center = detection.center;
            cv::putText(tframe.frame, "ID: " + std::to_string(detection.tag_id),
                    center, cv::FONT_HERSHEY_SIMPLEX, 3,
                    cv::Scalar(255, 0, 0), 5);
            
        }
    
        cv::imshow(windowName, fieldImg);
        cv::imshow("2", tframe.frame);
        if (PumpGuiEventsAndGetKey() == 'q') {
            break;
        }
    }
}
    
    
