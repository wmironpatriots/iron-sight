// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: TagSearcherIOOfficial.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/localization/TagSearcherIOAruco.h"

namespace localization {
    TagSearcherIOAruco::TagSearcherIOAruco() {
        auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        auto params = cv::aruco::DetectorParameters();

        detector_ = cv::aruco::ArucoDetector(dictionary, params);
    }

    auto TagSearcherIOAruco::FindTagsFromTimestampedFrame(const camera::timestamped_frame_t& tframe) -> std::vector<found_apriltag_t> {
        // Handle Empty Frame
        if (tframe.frame.empty()) {
            return {};
        }

        // Convert frame to grayscale
        cv::Mat gray_frame;
        cv::cvtColor(tframe.frame, gray_frame, cv::COLOR_BGR2GRAY); 

        // Create output vecs
        std::vector<int> marker_ids;
        std::vector<cv::Point2f> marker_center;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected;

        // Fill vecs
        detector_.detectMarkers(gray_frame, marker_corners, marker_ids, rejected);

        // Handle no detections
        if (marker_ids.empty()) return {};

        // Process output vecs into detection vec
        std::vector<found_apriltag_t> tags;
        for (int i = 0; i < marker_ids.size(); i++) {
            auto& tag = tags.emplace_back();

            tag.tag_id = marker_ids[i];
            tag.center_coords = marker_center[i];
            tag.timestamp_seconds = tframe.timestamp.value();

            for (int j = 0; j < 4; j++) {
                tag.corner_coords[j] = marker_corners[i][j];
            }

        }

        return tags;
    }
}
