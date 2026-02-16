// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimator.h
// Purpose: Define localization structs + Base PositionSolver class
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/utils/PCH.h"
#include "src/localization/AprilTagSearcher.h"

namespace localization {
    /** Represents a robot position estimate in 3D space (x, y, z) */
    using pose3d_estimate_t = struct Pose3dEstimate {
        /** A Pose3d object storing the estimated pose */
        frc::Pose3d position;
        /** The timestamp in seconds representing and estimate of when the robot was at this position */
        double timestamp;
        /** Represents the measured error in estimation */
        double variance;
    };

    /** Represents a class for estimating robot positions from found tags */
    class IPositionEstimator {
        public:
            virtual ~IPositionEstimator() = default;
            /** Return 3d position estimates from found tags */
            virtual auto estimatePosition(const std::vector<found_apriltag_t>& found_tags) -> std::vector<pose3d_estimate_t> = 0;
    };
}