// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CvAprilTagSearcher.h
// Purpose: Define AprilTagSearcher with OpenCV backend
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/localization/AprilTagSearcher.h"

namespace localization {
    // TODO
    class CvAprilTagSearcher : IAprilTagSearcher {
        public:
            CvAprilTagSearcher();
            auto findTags(camera::TimestampedFrame& tframe) -> std::vector<found_apriltag_t*> override;
    };
}
