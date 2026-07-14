// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: TagSearcherIOOfficial.h
// Purpose: Define a TagSearcher using the OpenCV's Aruco lib as a backend
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/localization/TagSearcherIO.h"
#include "src/camera/Camera.h"
#include "opencv2/aruco.hpp"

namespace localization {
    /** TagSearcherIO extension using the OpenCV's Aruco lib as backend */
    class TagSearcherIOAruco : public TagSearcherIO {
        public:
            TagSearcherIOAruco();
            auto FindTagsFromTimestampedFrame(const camera::timestamped_frame_t& tframe) -> found_apriltags_in_frame_t override;
        private:
            cv::aruco::ArucoDetector detector_;
    };
}
