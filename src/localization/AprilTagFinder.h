// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// TODO file + purpose
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include <apriltag/apriltag.h>
#include "src/utils/PCH.h"
#include "src/camera/Camera.h"

namespace localization {
    /** An interface for finding AprilTags in frames */
    class IAprilTagFinder {
        public:
            virtual ~IAprilTagFinder() = default;
            /** Returns a vector of detected april tags from a timestamped frame */
            virtual auto findTags(camera::TimestampedFrame& tframe) -> std::vector<apriltag_detection_t*>;
    };
}