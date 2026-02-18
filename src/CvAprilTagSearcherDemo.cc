// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CvAprilTagSearcherDemo.cc
// Purpose: Demo for CvAprilTagSearcher with camera integration
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/camera/Camera.h"
#include "src/camera/CameraCv.h"
#include "src/camera/CameraStream.h"
#include "src/localization/CvAprilTagSearcher.h"
#include "src/localization/MultiTagPositionEstimator.h"
//demo made from a lot of chatgpt bc i cant be bothered to make this myself
auto main() -> int {
    camera::CameraCv camera(camera::CameraConfig(0, cv::CAP_ANY));
    localization::CvAprilTagSearcher searcher;
    const std::string windowName = "AprilTag Detection Demo";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    while (true) {
        camera::TimestampedFrame tframe = camera.getTimestampedFrame();
        auto detections = searcher.findTags(tframe);

        for (const auto& detection : detections){
            std::vector<cv::Point2d> corners;
            corners.reserve(detection.cornerCoords.size());
        for (const auto& corner : detection.cornerCoords) {
                corners.push_back(corner);
            }
            std::vector<std::vector<cv::Point2d>> contours = {corners};
            cv::polylines(tframe.frame, contours, true, cv::Scalar(0, 255, 0), 2);
            for (auto & corner : corners) {
                cv::circle(tframe.frame, corner, 5, cv::Scalar(0, 0, 255), -1);
            }
            cv::Point2d center = detection.center;
            cv::putText(tframe.frame, "ID: " + std::to_string(detection.tag_id),
                       center, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(255, 0, 0), 2);
            
        }
        
        cv::imshow(windowName, tframe.frame);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    cv::destroyAllWindows();
    return 0;
}
