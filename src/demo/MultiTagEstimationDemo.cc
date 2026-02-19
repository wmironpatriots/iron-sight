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
#include "src/camera/CameraCv.h"
#include "src/camera/CameraStream.h"
#include "src/localization/CvAprilTagSearcher.h"
#include "src/localization/MultiTagPositionEstimator.h"
#include <cmath>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/version.hpp>
#include <chrono>
#include <thread>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>

namespace {
auto PumpGuiEventsAndGetKey() -> int {
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
    return cv::pollKey();
#else
    return cv::waitKey(1);
#endif
}
}

auto main() -> int {
    int camid;
    std::cout <<"Enter CamID: ";
    std::cin >> camid;
    camera::CameraCv camera(camera::CameraConfig(camid, cv::CAP_V4L2));
    localization::CvAprilTagSearcher searcher;
    const frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);

    int width = 1280;
    int height = 720;

    double fx = width;
    double fy = width;
    double cx = width / 2.0;
    double cy = height / 2.0;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1);

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    auto poseEstimator = localization::MultiTagPositionEstimator(fieldLayout, cameraMatrix, distCoeffs);

    const std::string windowName = "AprilTag Detection Demo";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    int count = 0;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        camera::TimestampedFrame tframe = camera.getTimestampedFrame();
        auto detections = searcher.findTags(tframe);
        auto pose = poseEstimator.estimatePosition(detections);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (count >= 5){
            std::cerr << "FPS: " << 1/elapsed.count() << "\n";
            std::cerr << pose[0].position.X().value() << " " << pose[0].position.Y().value() << " " << pose[0].position.Z().value() << "\n";
            count = 0;
        }
        count++;

        cv::Mat image = cv::imread("./resources/field.png");
        auto point = cv::Point2d(-(pose[0].position.Y().value() / 16.540988) * image.cols, (-pose[0].position.Z().value() / 8.069326) * tframe.frame.rows);
        cv::circle(image, point, 15, cv::Scalar(0, 0, 255), -1);

        cv::imshow(windowName, image);
        if (PumpGuiEventsAndGetKey() == 'q') {
            break;
        }
    }
}