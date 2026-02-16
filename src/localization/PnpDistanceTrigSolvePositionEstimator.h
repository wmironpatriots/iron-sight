// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PnpDistanceTrigSolvePositionEstimator.h
// Purpose: Define PositionEstimator with 6328's PNP distance + Trig solve backend
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/localization/AprilTagSearcher.h"
#include "src/localization/PositionEstimator.h"
#include <apriltag/frc/apriltag/AprilTagFieldLayout.h>

namespace localization {
    // TODO
    /** https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85 */
    class PnpDistanceTrigSolvePositionEstimator : IPositionEstimator {
        public:
            PnpDistanceTrigSolvePositionEstimator(frc::AprilTagFieldLayout layout);
            auto estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> override;
    };
}
