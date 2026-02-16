// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/localization/AprilTagFinder.h"

namespace localization {
    class AprilTagFinderCv : IAprilTagSearcher {
        public:
            // TODO
            AprilTagFinderCv();
            auto findTags(camera::TimestampedFrame& tframe) -> std::vector<found_apriltag_t*> override;
    };
}
