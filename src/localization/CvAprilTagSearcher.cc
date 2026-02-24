// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CvAprilTagSearcher.cc
// Purpose: Define april tag searcher
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/localization/CvAprilTagSearcher.h"
#include <apriltag.h>

namespace localization {
    CvAprilTagSearcher::CvAprilTagSearcher() {
        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
        td->quad_decimate = 2.0;
        td->quad_sigma = 0.0;
        td->nthreads = 4;
        td->refine_edges = true;
    };

    auto CvAprilTagSearcher::findTags(const camera::TimestampedFrame& tframe) -> std::vector<found_apriltag_t> {
        if (tframe.frame.empty()) {
            return {};
        }
        
        cv::Mat gray_frame;
        cv::cvtColor(tframe.frame, gray_frame, cv::COLOR_BGR2GRAY); 

        image_u8_t image {
        gray_frame.cols, 
        gray_frame.rows, 
        static_cast<int>(gray_frame.step), 
        gray_frame.data
        };
        
        zarray_t* raw_detections = apriltag_detector_detect(td, &image);
        const int detection_count = zarray_size(raw_detections);
        std::vector<found_apriltag_t> tag_detections{};
        tag_detections.reserve(static_cast<std::size_t>(detection_count));
        const double timestamp_seconds = tframe.timestamp.value();

        for (int i = 0; i < detection_count; i++){
            apriltag_detection_t* single_detection;
            zarray_get(raw_detections, i, &single_detection);
            
            auto& detection = tag_detections.emplace_back();
            detection.tag_id = single_detection->id;
            detection.decision_margin = single_detection->decision_margin;
            detection.center = cv::Point2d(single_detection->c[0], single_detection->c[1]);
            detection.timestampSeconds = timestamp_seconds;
            
            for (int j = 0; j < 4; j++){
                detection.cornerCoords[j] = cv::Point2d(single_detection->p[j][0], single_detection->p[j][1]);
            }
        }
        apriltag_detections_destroy(raw_detections);
        return tag_detections;
    }
    
    CvAprilTagSearcher::~CvAprilTagSearcher() {
        if (td != nullptr) {
            apriltag_detector_destroy(td);
            td = nullptr;
        }
        if (tf != nullptr) {
            tag36h11_destroy(tf);
            tf = nullptr;
        }
    }
}