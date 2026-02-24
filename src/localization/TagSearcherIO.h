// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: TagSearcher.h
// Purpose: Define the TagSearcherIO class and the structs it uses
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/utils/PCH.h"
#include "src/camera/Camera.h"

namespace localization {
    /** An april tag found within a frame */
    using found_apriltag_t = struct FoundAprilTag {
        /** Identity of Tag as a unique integer */
        int tag_id;
        /** Tag center coordinates in pixels */
        cv::Point2d center_coords;
        /** Tag corner coordinates in pixels */
        std::array<cv::Point2d, 4> corner_coords;
        /** When this tag was found */
        double timestamp_seconds;
        /** Measure of the quality of the binary decoding process: the average difference between the intensity of a data bit versus the decision threshold */
        float decision_margin;
    };

    /** Interface for finding tags within camera frames */
    class TagSearcherIO {
        public:
            virtual ~TagSearcherIO() = default;
            /** Returns a vector of found tags within a timestamped frame */
            virtual auto FindTagsFromTimestampedFrame(const camera::timestamped_frame_t& tframe) -> std::vector<found_apriltag_t> = 0;
    };
}