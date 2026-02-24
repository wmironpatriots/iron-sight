// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: TagSearcherIOWpilib.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/localization/TagSearcherIOWpiLib.h"

namespace localization {
    TagSearcherIOWpiLib::TagSearcherIOWpiLib() {
        tag_family_ = tag36h11_create();
        tag_detector_ = apriltag_detector_create();

        apriltag_detector_add_family(tag_detector_, tag_family_);

        tag_detector_->quad_decimate = 2.0;
        tag_detector_->quad_sigma = 0.0;
        tag_detector_->nthreads = 4;
        tag_detector_->refine_edges = true;
    };

    auto TagSearcherIOWpiLib::FindTagsFromTimestampedFrame(const camera::timestamped_frame_t& tframe) -> std::vector<found_apriltag_t> {
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
        
        zarray_t* raw_detections = apriltag_detector_detect(tag_detector_, &image);
        const int detection_count = zarray_size(raw_detections);
        std::vector<found_apriltag_t> tag_detections{};
        tag_detections.reserve(static_cast<std::size_t>(detection_count));
        const double timestamp_seconds = tframe.timestamp.value();


        for (int i = 0; i < detection_count; i++){
            apriltag_detection_t* single_detection;
            zarray_get(raw_detections, i, &single_detection);
            
            auto& detection = tag_detections.emplace_back();
            detection.tag_id = single_detection->id;
            detection.center_coords = cv::Point2d(single_detection->c[0], single_detection->c[1]);
            detection.timestamp_seconds = timestamp_seconds;
            
            for (int j = 0; j < 4; j++){
                detection.corner_coords[j] = cv::Point2d(single_detection->p[j][0], single_detection->p[j][1]);
            }
        }
        apriltag_detections_destroy(raw_detections);
        return tag_detections;
    }
    
    TagSearcherIOWpiLib::~TagSearcherIOWpiLib() {
        if (tag_detector_ != nullptr) {
            apriltag_detector_destroy(tag_detector_);
            tag_detector_ = nullptr;
        }
        if (tag_family_ != nullptr) {
            tag36h11_destroy(tag_family_);
            tag_family_ = nullptr;
        }
    }
}