// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// TODO file + purpose
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include <apriltag/apriltag.h>
#include <vector>
#include "src/camera/Camera.h"
#include "src/localization/AprilTagFinder.h"

namespace localization {
    class AprilTagFinderCv : IAprilTagFinder {
        public:
            AprilTagFinderCv();
            auto findTags(camera::TimestampedFrame& tframe) -> std::vector<apriltag_detection_t*> override;
    };
}
