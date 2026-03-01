#include "src/localization/PositionEstimatePublisher.h"
#include "src/localization/TagSearcherIOAruco.h"
#include "src/localization/TagSearcherIOWpiLib.h"
#include "src/utils/NtUtils.h"
#include "src/utils/PCH.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include <iostream>

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    0,
    cv::CAP_V4L2,
    "MJPG",
    1280,
    800,
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
/** Represents the program for the center cams unit */
auto main() -> int {
    camera::CameraIOCv camera(kDemoCam);

    auto searcher = localization::TagSearcherIOWpiLib();
    const frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);

    auto poseEstimator = localization::PositionEstimatorIOMultiTag(fieldLayout, kDemoCam);
    auto squarePoseEstimator = localization::PositionEstimatorIOSingleTag(fieldLayout, kDemoCam);

    utils::StartNetworkTables(true);
    auto publisher = localization::PositionEstimatePublisher(kDemoCam);
    
    while (true){
        camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
        
        auto detections = searcher.FindTagsFromTimestampedFrame(tframe);
        //TODO add check for how many apriltags are seen    

        auto pose = poseEstimator.Estimate3dPoseFromFoundTags(detections);
        auto squarepose = squarePoseEstimator.Estimate3dPoseFromFoundTags(detections);
        
        //TODO cache poses to publisher to not overwrite here, also differentiate from single/multi in one publisher
        if (!squarepose.empty()) publisher.Publish(squarepose[0]);
        if (!pose.empty()) publisher.Publish(pose[0]);

    }
    return 0;
}
