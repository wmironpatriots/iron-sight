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
    A Script for the O̶r̶a̶n̶g̶e Rubik Pi Unit (thanks dasun) connected to the Front and Back
    Camera of FRC 6423's 2026 Robot
*/

inline const frc::AprilTagFieldLayout kFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);

// TODO intrinsics
inline const camera::camera_config_t kBessieConfig = camera::camera_config_t{
    "bessie",
    "/dev/v4l/by-path/platform-xhci-hcd.0.auto-usbv2-0:1:1.0-video-index0",
    cv::CAP_V4L,
    "MJPG",
    800,
    600,
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
inline const camera::camera_config_t kElsieConfig = camera::camera_config_t{
    "elsie",
    "/dev/v4l/by-path/platform-1c00000.pci-pci-0000:01:00.0-usb-0:1:1.0-video-index0",
    cv::CAP_V4L2,
    "MJPG",
    800,
    600,
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

/** Entry Point */
auto main() -> int {
    utils::StartNetworkTables(false);

    camera::camera_config_t BessieConfig = kBessieConfig;
    camera::camera_config_t ElsieConfig = kElsieConfig;

    /* ~ CAMERA INIT ~ */
    std::printf("Initializing Front Camera (Bessie)\n");

    camera::CameraIOCv frontCamera(BessieConfig);
    camera::CameraIOCv backCamera(ElsieConfig);

    // TODO Elsie Init

    /* ~ SEARCHER/ESTIMATOR INIT ~ */
    auto frontSearcher = localization::TagSearcherIOWpiLib();

    auto frontPoseEstimator = localization::PositionEstimatorIOCombined(kFieldLayout, kBessieConfig);
    
    auto backSearcher = localization::TagSearcherIOWpiLib();

    auto backPoseEstimator = localization::PositionEstimatorIOCombined(kFieldLayout, kElsieConfig);

    /* ~ PUBLISHER INIT ~ */
    auto frontPublisher = localization::PositionEstimatePublisher(kBessieConfig);
    auto backPublisher = localization::PositionEstimatePublisher(kElsieConfig);
    /* ~ THREAD INIT ~ */
    std::thread front_thread([&frontCamera, &frontSearcher, &frontPoseEstimator, &frontPublisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = frontCamera.GetTimestampedFrame();

            auto detections = frontSearcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = frontPoseEstimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                frontPublisher.Publish(pose[0], latency.count());
            }
        }
    });

        std::thread back_thread([&backCamera, &backSearcher, &backPoseEstimator, &backPublisher] () -> void {
        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            camera::timestamped_frame_t tframe = backCamera.GetTimestampedFrame();

            auto detections = backSearcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = backPoseEstimator.Estimate3dPoseFromFoundTags(detections);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> latency = end - start;
            
            if (!pose.empty()) {
                backPublisher.Publish(pose[0], latency.count());
            }
        }
    });

    back_thread.join();
    return 0;
}
