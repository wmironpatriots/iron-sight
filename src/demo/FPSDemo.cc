#include "src/localization/TagSearcherIOAruco.h"
#include "src/utils/PCH.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include "src/localization/TagSearcherIOWpiLib.h"

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    "/dev/v4l/by-path/platform-xhci-hcd.0.auto-usbv2-0:1:1.0-video-index0",
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

    localization::TagSearcherIOAruco searcher;
    const frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);

    auto poseEstimator = localization::PositionEstimatorIOMultiTag(fieldLayout, kDemoCam);
    auto squarePoseEstimator = localization::PositionEstimatorIOSingleTag(fieldLayout, kDemoCam);

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
        auto detections = searcher.FindTagsFromTimestampedFrame(tframe);
        auto pose = poseEstimator.Estimate3dPoseFromFoundTags(detections);
        auto squarepose = squarePoseEstimator.Estimate3dPoseFromFoundTags(detections);

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