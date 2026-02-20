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
    class CvAprilTagSearcher : public IAprilTagSearcher {
        public:
            CvAprilTagSearcher();
            ~CvAprilTagSearcher() override;
            CvAprilTagSearcher(const CvAprilTagSearcher&) = delete;
            auto operator=(const CvAprilTagSearcher&) -> CvAprilTagSearcher& = delete;
            CvAprilTagSearcher(CvAprilTagSearcher&&) = delete;
            auto operator=(CvAprilTagSearcher&&) -> CvAprilTagSearcher& = delete;
            auto findTags(const camera::TimestampedFrame& tframe) -> std::vector<found_apriltag_t> override;
        private:
            apriltag_family_t* tf;
            apriltag_detector_t* td;
    };
}
