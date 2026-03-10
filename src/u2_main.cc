#include <frc/geometry/Rotation3d.h>
#include <wpimath/frc/geometry/Transform3d.h>
#include <cstdio>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <thread>
#include "src/localization/PositionEstimatePublisher.h"
#include "src/localization/PositionEstimatorIOCombined.h"
#include "src/localization/TagSearcherIOAruco.h"
#include "src/localization/TagSearcherIOWpiLib.h"
#include "src/utils/NtUtils.h"
#include "src/utils/PCH.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include <units/length.h>

/**
    A Script for the Raspberry Pi Unit connected to the Left and Right
    Camera of FRC 6423's 2026 Robot
*/

inline const frc::AprilTagFieldLayout kFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);

// TODO intrinsics
inline const camera::camera_config_t kBeatriceConfig = camera::camera_config_t{
    "beatrice",
    "/dev/v4l/by-path/platform-xhci-hcd.0.auto-usbv2-0:1:1.0-video-index0",
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    120,
    frc::Transform3d(
        -12.255_in, 
        0.0_in, 
        14.207_in, 
        frc::Rotation3d(
            0.0_rad,
            -0.523599_rad,
            0.0_rad
        )
    ),
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
// TODO extrinsics & intrinsics
inline const camera::camera_config_t kBelindaConfig = camera::camera_config_t{
    "belinda",
    "/dev/v4l/by-path/platform-1c00000.pci-pci-0000:01:00.0-usbv2-0:1:1.0-video-index0",
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    120,
    frc::Transform3d(
        7.54_in, 
        1.5_in, 
        20.094_in, 
        frc::Rotation3d(
            0.0_rad,
            0.261799_rad,
            0.0_rad
        )
    ),
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
    utils::StartNetworkTables(false);

    camera::camera_config_t BeatriceConfig = kBeatriceConfig;
    camera::camera_config_t BelindaConfig = kBelindaConfig;
    /* ~ CAMERA INIT ~ */
    std::printf("Initializing Front Camera (Bessie)");
    camera::CameraIOCv rightCamera(BeatriceConfig);
    camera::CameraIOCv leftCamera(BelindaConfig);
    // TODO Elsie Init

    /* ~ SEARCHER/ESTIMATOR INIT ~ */
    auto rightSearcher = localization::TagSearcherIOWpiLib();

    auto rightPoseEstimator = localization::PositionEstimatorIOCombined(kFieldLayout, kBeatriceConfig);
    
    auto leftSearcher = localization::TagSearcherIOWpiLib();

    auto leftPoseEstimator = localization::PositionEstimatorIOCombined(kFieldLayout, kBelindaConfig);

    /* ~ PUBLISHER INIT ~ */
    auto rightPublisher = localization::PositionEstimatePublisher(kBeatriceConfig);
    auto leftPublisher = localization::PositionEstimatePublisher(kBelindaConfig);
    /* ~ THREAD INIT ~ */
    std::thread right_thread([&rightCamera, &rightSearcher, &rightPoseEstimator, &rightPublisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = rightCamera.GetTimestampedFrame();

            auto detections = rightSearcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = rightPoseEstimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                rightPublisher.Publish(pose[0], latency.count());
            }
        }
    });

        std::thread left_thread([&leftCamera, &leftSearcher, &leftPoseEstimator, &leftPublisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = leftCamera.GetTimestampedFrame();

            auto detections = leftSearcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = leftPoseEstimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                leftPublisher.Publish(pose[0], latency.count());
            }
        }
    });

    left_thread.join();
    return 0;
}
