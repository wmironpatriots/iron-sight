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
#include "src/camera/CameraIOCv.h"
#include "src/camera/CameraStream.h"
#include "src/localization/CvAprilTagSearcher.h"
#include "src/localization/MultiTagPositionEstimator.h"
#include "src/localization/IPPESquarePositionEstimator.h"
#include "src/localization/PositionEstimatePublisher.h"
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

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
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
    camera::CameraIOCv camera(kDemoCam);

    localization::CvAprilTagSearcher searcher;
    const frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);

    auto poseEstimator = localization::MultiTagPositionEstimator(fieldLayout, kDemoCam);
    auto squarePoseEstimator = localization::IPPESquarePositionEstimator(fieldLayout, kDemoCam);

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
        auto detections = searcher.findTags(tframe);
        auto pose = poseEstimator.estimatePosition(detections);
        auto squarepose = squarePoseEstimator.estimatePosition(detections);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::system("clear");
        std::cerr << "FPS: " << 1/elapsed.count() << "\n";
        if (!pose.empty()){
            std::cerr << pose[0].position.X().value() << "\n";
            std::cerr << pose[0].position.Y().value() << "\n"; 
            std::cerr << pose[0].position.Z().value() << "\n";
                
        }
    }
}