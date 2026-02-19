// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CvAprilTagSearcherDemo.cc
// Purpose: Demo for MultiTagPositionEstimator
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include <frc/apriltag/AprilTagFields.h>
#include "src/camera/Camera.h"
#include "src/camera/CameraCv.h"
#include "src/camera/CameraStream.h"
#include "src/localization/CvAprilTagSearcher.h"
#include "src/localization/MultiTagPositionEstimator.h"
#include <cmath>
#include <opencv2/core/version.hpp>
#include <frc/Timer.h>

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
    frc::Timer timer;
    int count = 0;
    while (true) {
        timer.Start();
        camera::TimestampedFrame tframe = camera.getTimestampedFrame();
        auto detections = searcher.findTags(tframe);
        auto pose = poseEstimator.estimatePosition(detections);
        timer.Stop();
        if (count >= 30){
            units::second_t time = timer.Get();
            std::cerr << "FPS: " << 1/time.value() << "\n";
            std::cerr << pose[0].position.X().value() << " " << pose[0].position.Y().value() << " " << pose[0].position.Z().value() << "\n";
            count = 0;
        }
        count++;
        timer.Reset();
    }
}