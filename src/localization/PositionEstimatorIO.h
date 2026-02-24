// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimatorIO.h
// Purpose: Define the PositionEstimatorIO class and the structs it uses
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/utils/PCH.h"
#include "src/localization/TagSearcherIO.h"
#include <opencv2/calib3d.hpp>

namespace localization {
    /** Sidge Length of Tags */
    constexpr units::length::meter_t kTagSideLength{0.1651};

    /** Coordinates of Tag Corners (TL, TR, BR, BL) */
    const std::vector<frc::Pose3d> kTagCorners = {
        frc::Pose3d(kTagSideLength / -2, kTagSideLength / 2, 0_m, frc::Rotation3d()),
        frc::Pose3d(kTagSideLength / 2, kTagSideLength / 2, 0_m, frc::Rotation3d()),
        frc::Pose3d(kTagSideLength / 2, kTagSideLength / -2, 0_m, frc::Rotation3d()),
        frc::Pose3d(kTagSideLength / -2, kTagSideLength / -2, 0_m, frc::Rotation3d())
    };

    /** An estimated position in 3-Dimensional space /w a confidence value */
    using pose3d_estimate_t = struct Pose3dEstimate {
        /** Tags used to derive the estimate */
        std::vector<found_apriltag_t> tags;
        /** The position estimated in 3-Dimensional space */
        frc::Pose3d position;
        /** When the robot was estimated to be in this position */
        double timestamp;
        /** The measured error in pixels */
        double variance;
    };

    /** Interface for estimating the robot position from found tags */
    class PositionEstimatorIO {
        public:
            virtual ~PositionEstimatorIO() = default;
            /** Returns 3-Dimensional position estimates from a vector of found tags */
            virtual auto Estimate3dPoseFromFoundTags(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> = 0;
    };
}