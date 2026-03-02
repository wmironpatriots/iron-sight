#include <frc/geometry/Rotation3d.h>
#include <wpimath/frc/geometry/Transform3d.h>
#include <cstdio>
#include <thread>
#include "src/localization/PositionEstimatePublisher.h"
#include "src/localization/TagSearcherIOAruco.h"
#include "src/localization/TagSearcherIOWpiLib.h"
#include "src/utils/NtUtils.h"
#include "src/utils/PCH.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include <units/length.h>

/**
    A Script for the Orange Pi Unit connected to the Front and Back
    Camera of FRC 6423's 2026 Robot
*/

inline const frc::AprilTagFieldLayout kFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);

// TODO intrinsics
inline const camera::camera_config_t kBessieConfig = camera::camera_config_t{
    "bessie",
    0,
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
            0.0_rad,
            0.0_rad
        )
    ),
    camera::camera_intrinsics_t{
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    }
};
// TODO extrinsics & intrinsics
inline const camera::camera_config_t kElsieConfig = camera::camera_config_t{
    "elsie",
    1,
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
    120,
    frc::Transform3d(
        0.0_in, 
        0.0_in, 
        0.0_in, 
        frc::Rotation3d(
            0.0_rad,
            0.0_rad,
            0.0_rad
        )
    ),
    camera::camera_intrinsics_t{
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    }
};

/** Entry Point */
auto main() -> int {
    utils::StartNetworkTables(true);

    /* ~ CAMERA INIT ~ */
    std::printf("Initializing Front Camera (Bessie)");
    camera::CameraIOCv camera(kBessieConfig);

    // TODO Elsie Init

    /* ~ SEARCHER/ESTIMATOR INIT ~ */
    auto searcher = localization::TagSearcherIOWpiLib();

    auto poseEstimator = localization::PositionEstimatorIOMultiTag(kFieldLayout, kBessieConfig);
    auto squarePoseEstimator = localization::PositionEstimatorIOSingleTag(kFieldLayout, kBessieConfig);

    /* ~ PUBLISHER INIT ~ */
    auto publisher = localization::PositionEstimatePublisher(kBessieConfig);
    
    /* ~ THREAD INIT ~ */
    std::thread front_thread([&camera, &searcher, &poseEstimator, &squarePoseEstimator, &publisher] () -> void {
        while (true) {
            camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
            
            auto detections = searcher.FindTagsFromTimestampedFrame(tframe);

            auto pose = poseEstimator.Estimate3dPoseFromFoundTags(detections);
            auto squarepose = squarePoseEstimator.Estimate3dPoseFromFoundTags(detections);

            if (squarepose.size() == 1) publisher.Publish(squarepose[0]);
            if (!pose.empty()) publisher.Publish(pose[0]);
        }
    });

    front_thread.join();

    return 0;
}
