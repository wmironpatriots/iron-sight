// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: u1_main.cc
// Purpose: A Script for the O̶r̶a̶n̶g̶e Rubik Pi Unit (thanks dasun) connected to the Front and Back
//          Camera of FRC 6423's 2026 Robot
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/utils/PCH.h"
#include "src/utils/NtUtils.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/localization/TagSearcherIOWpiLib.h"
#include "src/localization/PositionEstimatePublisher.h"
#include "src/localization/PositionEstimatorIOCombined.h"

// * ~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~

/* Apriltag field layout to use */
inline const frc::AprilTagFieldLayout FIELD_LAYOUT = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);

/* Configuration for Elsie; the front camera */
//og  /dev/v4l/by-path/platform-1c00000.pci-pci-0000:01:00.0-usb-0:1:1.0-video-index0
//now, top blue port
inline const camera::camera_config_t ELSIE_CONFIG = camera::camera_config_t{
    "elsie",
    "/dev/v4l/by-path/platform-1c00000.pci-pci-0000:01:00.0-usbv2-0:2:1.0-video-index0",
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    120,
    camera::camera_intrinsics_t{
        625.3426025032783,
        372.4797842477203,
        898.0928868060495,
        897.7157683218877,
        0.04705864839856624,
        -0.08074506402566872,
        0.01743537811199019,
        0.001136572471268671,
        -0.0003280265291867128,
    }
};

/* Configuration for Bessie; the back camera */
// middle hub port
inline const camera::camera_config_t BESSIE_CONFIG = camera::camera_config_t{
    "bessie",
    "/dev/v4l/by-path/platform-1c00000.pci-pci-0000:01:00.0-usbv2-0:1.2:1.0-video-index0",
    cv::CAP_V4L,
    "MJPG",
    1280,
    800,
    120,
    camera::camera_intrinsics_t{
        625.3426025032783,
        372.4797842477203,
        898.0928868060495,
        897.7157683218877,
        0.04705864839856624,
        -0.08074506402566872,
        0.01743537811199019,
        0.001136572471268671,
        -0.0003280265291867128,
    }
};

/* Configuration for Beatrice; the right camera, furthest to hub wire */
inline const camera::camera_config_t BEATRICE_CONFIG = camera::camera_config_t{
    "beatrice",
    "/dev/v4l/by-path/platform-1c00000.pci-pci-0000:01:00.0-usbv2-0:1.3:1.0-video-index0",
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    120,
    camera::camera_intrinsics_t{
        625.3426025032783,
        372.4797842477203,
        898.0928868060495,
        897.7157683218877,
        0.04705864839856624,
        -0.08074506402566872,
        0.01743537811199019,
        0.001136572471268671,
        -0.0003280265291867128,
    }
};

/* Configuration for Belinda; the left camera, closest to hub wire */
/* THIS IS THE BAD FPS CAMERA */
inline const camera::camera_config_t BELINDA_CONFIG = camera::camera_config_t{
    "belinda",
    "/dev/v4l/by-path/platform-1c00000.pci-pci-0000:01:00.0-usbv2-0:1.1:1.0-video-index0",
    cv::CAP_V4L2,
    "MJPG",
    1280,
    720,
    50,
    camera::camera_intrinsics_t{
        625.3426025032783,
        372.4797842477203,
        898.0928868060495,
        897.7157683218877,
        0.04705864839856624,
        -0.08074506402566872,
        0.01743537811199019,
        0.001136572471268671,
        -0.0003280265291867128,
    }
};

/** Entry Point */
auto main() -> int {
    // * ~~~~~~~~~~~~~ INITIAL CONFIGURATION ~~~~~~~~~~~~~

    utils::StartNetworkTables(false);

    // * ~~~~~~~~~~~~~ ELSIE (FRONT CAMERA) SETUP ~~~~~~~~~~~~~

    std::printf("Initializing Elsie (Front Camera)\n");

    camera::camera_config_t elsie_config = ELSIE_CONFIG;
    camera::CameraIOCv front_camera(elsie_config);
    
    auto front_searcher = localization::TagSearcherIOWpiLib();
    auto front_pose_estimator = localization::PositionEstimatorIOCombined(FIELD_LAYOUT, ELSIE_CONFIG);

    auto front_nt_publisher = localization::PositionEstimatePublisher(ELSIE_CONFIG);

    std::printf("Elsie started successfully!\n");

    // * ~~~~~~~~~~~~~ BESSIE (BACK CAMERA) SETUP ~~~~~~~~~~~~~

    std::printf("Initializing Bessie (Back Camera)\n");

    camera::camera_config_t bessie_config = BESSIE_CONFIG;
    camera::CameraIOCv back_camera(bessie_config);

    auto back_searcher = localization::TagSearcherIOWpiLib();
    auto back_pose_estimator = localization::PositionEstimatorIOCombined(FIELD_LAYOUT, BESSIE_CONFIG);

    auto back_nt_publisher = localization::PositionEstimatePublisher(BESSIE_CONFIG);

    std::printf("Bessie started successfully!\n");

    // * ~~~~~~~~~~~~~ BEATRICE (RIGHT CAMERA) SETUP ~~~~~~~~~~~~~

    std::printf("Initializing Beatrice (Right Camera)\n");

    camera::camera_config_t beatrice_config = BEATRICE_CONFIG;
    camera::CameraIOCv right_camera(beatrice_config);

    auto right_searcher = localization::TagSearcherIOWpiLib();
    auto right_pose_estimator = localization::PositionEstimatorIOCombined(FIELD_LAYOUT, BEATRICE_CONFIG);

    auto right_nt_publisher = localization::PositionEstimatePublisher(BEATRICE_CONFIG);

    std::printf("Beatrice started successfully!\n");

    // * ~~~~~~~~~~~~~ BELINDA (LEFT CAMERA) SETUP ~~~~~~~~~~~~~

    std::printf("Initializing Belinda (Left Camera)\n");

    camera::camera_config_t belinda_config = BELINDA_CONFIG;
    camera::CameraIOCv left_camera(belinda_config);
    
    auto left_searcher = localization::TagSearcherIOWpiLib();
    auto left_pose_estimator = localization::PositionEstimatorIOCombined(FIELD_LAYOUT, BELINDA_CONFIG);

    auto left_nt_publisher = localization::PositionEstimatePublisher(BELINDA_CONFIG);

    std::printf("Belinda started successfully!\n");

    // * ~~~~~~~~~~~~~ THREAD INIT ~~~~~~~~~~~~~

    std::thread back_thread([&back_camera, &back_searcher, &back_pose_estimator, &back_nt_publisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = back_camera.GetTimestampedFrame();

            auto detections = back_searcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = back_pose_estimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                back_nt_publisher.Publish(pose[0], latency.count());
            }
        }
    });

    std::thread front_thread([&front_camera, &front_searcher, &front_pose_estimator, &front_nt_publisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = front_camera.GetTimestampedFrame();

            auto detections = front_searcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = front_pose_estimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                front_nt_publisher.Publish(pose[0], latency.count());
            }
        }
    });

    std::thread right_thread([&right_camera, &right_searcher, &right_pose_estimator, &right_nt_publisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = right_camera.GetTimestampedFrame();

            auto detections = right_searcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = right_pose_estimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                right_nt_publisher.Publish(pose[0], latency.count());
            }
        }
    });

    std::thread left_thread([&left_camera, &left_searcher, &left_pose_estimator, &left_nt_publisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = left_camera.GetTimestampedFrame();

            auto detections = left_searcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = left_pose_estimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                left_nt_publisher.Publish(pose[0], latency.count());
            }
        }
    });

    left_thread.join();

    return 0;
}
