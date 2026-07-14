// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimatorIO.h
// Purpose: Define the PositionEstimatorIO class and the structs it uses
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include <units/length.h>
#include "src/utils/PCH.h"
#include "src/localization/TagSearcherIO.h"
#include <opencv2/calib3d.hpp>

namespace localization {
    /** Side Length of Tags */
    constexpr auto kTagSideLength = 0.1651;

    /** Coordinates of Tag Corners (BR, BL, TL, TR) */
    const std::vector<cv::Vec3f> kTagCorners = {
        cv::Vec3f(-kTagSideLength/2.f, kTagSideLength/2.f, 0),
        cv::Vec3f(kTagSideLength/2.f, kTagSideLength/2.f, 0),
        cv::Vec3f(kTagSideLength/2.f, -kTagSideLength/2.f, 0),
        cv::Vec3f(-kTagSideLength/2.f, -kTagSideLength/2.f, 0)
    };

    /** An estimated position in 3-Dimensional space /w a confidence value */
    using pose3d_estimate_t = struct Pose3dEstimate {
        /** The position estimated in 3-Dimensional space */
        frc::Pose3d position;
        /** When the robot was estimated to be in this position */
        double timestamp;
        /** The measured error in pixels */
        double variance;
        /** The number of tags used to calculate this position */
        int tagsUsed;
    };

    /** Interface for estimating the robot position from found tags */
    class PositionEstimatorIO {
        public:
            virtual ~PositionEstimatorIO() = default;
            /** Returns 3-Dimensional position estimates from a vector of found tags */
            virtual auto Estimate3dPoseFromFoundTags(const found_apriltags_in_frame_t& found_tags) -> std::vector<pose3d_estimate_t> = 0;
    };
}