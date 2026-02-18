// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: AprilTagSearcher.h
// Purpose: Define base AprilTagSearcher structs & class
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/utils/PCH.h"
#include "src/camera/Camera.h"
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

namespace localization {
    /** Represents a found apriltag from a tframe */
    using found_apriltag_t = struct FoundAprilTag {
        /** The unique id of tag */
        int tag_id;
        /** An array of each tag corner coordinate in pixels */
        std::array<cv::Point2d, 4> cornerCoords;
        /** The timestamp in seconds representing when this tag was viewed at this angle aka the timestamp of the timestamped_frame_t it was derived from */
        double timestampSeconds;
        /**The decision margin, "A measure of the quality of the binary decoding process: the average difference between the intensity of a data bit versus the decision threshold." */
        float decision_margin;
        /**The center of the detection in image pixel coordinates*/
        cv::Point2d center;
    };

    /** An interface for finding AprilTags in frames */
    class IAprilTagSearcher {
        public:
            virtual ~IAprilTagSearcher() = default;
            /** Returns a vector of found april tags from a timestamped frame */
            virtual auto findTags(camera::TimestampedFrame& tframe) -> std::vector<found_apriltag_t>;
    };
}