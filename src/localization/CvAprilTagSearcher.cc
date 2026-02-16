// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CvAprilTagSearcher.cc
// Purpose: Define april tag searcher
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/localization/CvAprilTagSearcher.h"

namespace localization {
    CvAprilTagSearcher::CvAprilTagSearcher(){}

    auto CvAprilTagSearcher::findTags(camera::TimestampedFrame& tframe) -> std::vector<found_apriltag_t*> {
        apriltag_family_t *tf = tag36h11_create();
        apriltag_detector_t *td = apriltag_Detector_create();

        apriltag_detector_add_family(td, tf);
        
        td->quad_decimate = 2.0;
        td->quad_sigma = 0.0;
        td->nthreads = 4;
        cv::Mat frame = tframe->frame;
        image_u8_t image = {frame.cols, frame.rows, frame.cols, frame.data};
        
        zarray_t* raw_detections = apriltag_detector_detect(td, &image);

        std::vector<found_apriltag_t*> tag_detections{};

        for (int i = 0; i < zarray_size(raw_detections); i++){
            apriltag_detection_t* single_detection;
            zarray_get(raw_detections, i, &single_detection);
            
            found_apriltag_t detection;
            detection.tag_id = single_detection->id;
            detection.decision_margin = single_detection->decision_margin;
            detection.center = single_detection->c
            detection.timestampSeconds = tframe->timestamp;
            
            for (int j = 0; j < 4; j++){
                detection.cornerCoords[j] = cv::Point2d(single_detection->p[j][0], single_detection->p[j][0])
            }
            tag_detections.push_back(detection);
        }
    }
    return tag_detections;
}