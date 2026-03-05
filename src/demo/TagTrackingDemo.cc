#include "src/localization/TagSearcherIOAruco.h"
#include "src/utils/PCH.h"
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include "src/localization/PositionEstimatorIOMultiTag.h"
#include "src/localization/PositionEstimatorIOSingleTag.h"
#include "src/localization/TagSearcherIOWpiLib.h"

inline const camera::camera_config_t kDemoCam = camera::camera_config_t{
    "bessie",
    "example",
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
    std::cout << "Enter camid: ";
    std::cin >> camid;
    camera::CameraIOCv camera(kDemoCam);
    localization::TagSearcherIOAruco searcher;

    const std::string windowName = "AprilTag Detection Demo";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    while (true) {

        camera::timestamped_frame_t tframe = camera.GetTimestampedFrame();
        auto detections = searcher.FindTagsFromTimestampedFrame(tframe);

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
        cv::imshow(windowName, tframe.frame);
        if (PumpGuiEventsAndGetKey() == 'q') {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
