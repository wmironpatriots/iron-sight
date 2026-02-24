#include "src/localization/PositionEstimatePublisher.h"
#include "src/localization/TagSearcherIOAruco.h"
#include "src/utils/NtUtils.h"
#include "src/utils/PCH.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/localization/PositionEstimatorIOSingleTag.h"

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
    "bessie",
    2,
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

auto main() -> int {
    camera::CameraIOCv camera(kDemoCam);

    auto searcher = localization::TagSearcherIOAruco();
    const frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);

    auto poseEstimator = localization::PositionEstimatorIOMultiTag(fieldLayout, kDemoCam);
    auto squarePoseEstimator = localization::PositionEstimatorIOSingleTag(fieldLayout, kDemoCam);
    const std::string windowName = "AprilTag Detection Demo";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::namedWindow("2", cv::WINDOW_NORMAL);

    utils::StartNetworkTables(true);
    auto publisher = localization::PositionEstimatePublisher(kDemoCam);
    auto squarePublisher = localization::PositionEstimatePublisher(kDemoCam);

    int count = 0;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
        auto detections = searcher.FindTagsFromTimestampedFrame(tframe);
        auto pose = poseEstimator.Estimate3dPoseFromFoundTags(detections);
        auto squarepose = squarePoseEstimator.Estimate3dPoseFromFoundTags(detections);
        if (!pose.empty()) publisher.Publish(pose[0]);
        if (!squarepose.empty()) publisher.Publish(squarepose[0]);

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
        }

        for (const auto& detection : detections){
            std::vector<cv::Point> corners;
            corners.reserve(detection.corner_coords.size());
        for (const auto& corner : detection.corner_coords) {
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
            cv::Point2d center = detection.center_coords;
            cv::putText(tframe.frame, "ID: " + std::to_string(detection.tag_id),
                    center, cv::FONT_HERSHEY_SIMPLEX, 3,
                    cv::Scalar(255, 0, 0), 5);
            
        }
    
        cv::imshow("2", tframe.frame);
        if (PumpGuiEventsAndGetKey() == 'q') {
            break;
        }
    }
}
    
    
