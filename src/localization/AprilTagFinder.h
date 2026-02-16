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
#include "src/localization/PositionEstimator.h"
#include "src/camera/Camera.h"

namespace localization {
    /** An interface for finding AprilTags in frames */
    class IAprilTagSearcher {
        public:
            virtual ~IAprilTagSearcher() = default;
            /** Returns a vector of found april tags from a timestamped frame */
            virtual auto findTags(camera::TimestampedFrame& tframe) -> std::vector<localiztaion::found_apriltag_t*>;
    };
}