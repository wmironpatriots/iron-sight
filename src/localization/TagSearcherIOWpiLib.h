// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: TagSearcherIOOfficial.h
// Purpose: Define a TagSearcher using the WPIlib AprilTag lib as a backend
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/localization/TagSearcherIO.h"
#include "src/camera/Camera.h"

namespace localization {
    /** TagSearcherIO extension using the WPIlib AprilTag lib as backend */
    class TagSearcherIOWpiLib : public TagSearcherIO {
        public:
            TagSearcherIOWpiLib();
            ~TagSearcherIOWpiLib() override;
            auto FindTagsFromTimestampedFrame(const camera::timestamped_frame_t& tframe) -> std::vector<found_apriltag_t> override;
        private:
            apriltag_family_t* tag_family_;
            apriltag_detector_t* tag_detector_;
    };
}
