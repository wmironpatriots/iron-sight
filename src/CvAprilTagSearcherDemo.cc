// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CvAprilTagSearcherDemo.cc
// Purpose: Demo for CvAprilTagSearcher with camera integration
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include <frc/Timer.h>
#include "src/camera/Camera.h"
#include "src/camera/CameraCv.h"
#include "src/camera/CameraStream.h"
#include "src/localization/CvAprilTagSearcher.h"
#include "src/localization/MultiTagPositionEstimator.h"
#include <cmath>
#include <opencv2/core/version.hpp>

// namespace {
// auto PumpGuiEventsAndGetKey() -> int {
// #if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
//     return cv::pollKey();
// #else
//     return cv::waitKey(1);
// #endif
// }
// }

//demo made from a lot of chatgpt bc i cant be bothered to make this myself
auto main() -> int {
    camera::CameraCv camera(camera::CameraConfig(2, cv::CAP_V4L2));
    localization::CvAprilTagSearcher searcher;

    auto timer = frc::Timer();

    // const std::string windowName = "AprilTag Detection Demo";
    // cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    while (true) {

        camera::TimestampedFrame tframe = camera.getTimestampedFrame();
        auto detections = searcher.findTags(tframe);

        for (const auto& detection : detections){
            std::vector<cv::Point> corners;
            corners.reserve(detection.cornerCoords.size());
        for (const auto& corner : detection.cornerCoords) {
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
            cv::Point2d center = detection.center;
            cv::putText(tframe.frame, "ID: " + std::to_string(detection.tag_id),
                       center, cv::FONT_HERSHEY_SIMPLEX, 3,
                       cv::Scalar(255, 0, 0), 5);
            
        }
        // cv::imshow(windowName, tframe.frame);
        // if (PumpGuiEventsAndGetKey() == 'q') {
        //     break;
        // }
    }

    // cv::destroyAllWindows();
    return 0;
}
