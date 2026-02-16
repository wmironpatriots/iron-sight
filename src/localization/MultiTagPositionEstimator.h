// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: MultiTagPositionEstimator.h
// Purpose: Define PositionEstimator with multitag estimation backend
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/localization/PositionEstimator.h"
#include "apriltag/frc/apriltag/AprilTagFieldLayout.h"

namespace localization {
    // TODO
    /** Represents a PositionEstimator that solves based on many found tags */
    class MultiTagPositionEstimator : IPositionEstimator {
        public:
            MultiTagPositionEstimator(const frc::AprilTagFieldLayout& fieldLayout);
            auto estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> override;
    };
}
