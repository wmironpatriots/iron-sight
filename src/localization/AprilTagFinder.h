// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: AprilTagFinder.h
// Purpose: Define base AprilTagFinder class
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/utils/PCH.h"
#include "src/camera/Camera.h"

namespace localization {
    /** Represents a found apriltag from a tframe */
    using found_apriltag_t = struct FoundAprilTag {
        /** The unique id of tag */
        int tag_id;
        /** An array of each tag corner coordinate in pixels */
        std::array<cv::Point2d, 4> cornerCoords;
        /** The timestamp in seconds representing when this tag was viewed at this angle aka the timestamp of the timestamped_frame_t it was derived from */
        double timestampSeconds;
    };

    /** An interface for finding AprilTags in frames */
    class IAprilTagSearcher {
        public:
            virtual ~IAprilTagSearcher() = default;
            /** Returns a vector of found april tags from a timestamped frame */
            virtual auto findTags(camera::TimestampedFrame& tframe) -> std::vector<found_apriltag_t*>;
    };
}