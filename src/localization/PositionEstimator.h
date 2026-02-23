// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimator.h
// Purpose: Define localization structs + Base PositionSolver class
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <vector>
#include "src/utils/PCH.h"
#include "src/localization/AprilTagSearcher.h"
#include "units/length.h"

namespace localization {
    constexpr units::length::meter_t kTagSize{0.1651};
    const std::vector<frc::Pose3d> kTagCorners = {
        frc::Pose3d(kTagSize / -2, kTagSize / 2, 0_m, frc::Rotation3d()),
        frc::Pose3d(kTagSize / 2, kTagSize / 2, 0_m, frc::Rotation3d()),
        frc::Pose3d(kTagSize / 2, kTagSize / -2, 0_m, frc::Rotation3d()),
        frc::Pose3d(kTagSize / -2, kTagSize / -2, 0_m, frc::Rotation3d())
    };

    /** Represents a robot position estimate in 3D space (x, y, z) */
    using pose3d_estimate_t = struct Pose3dEstimate {
        /** The tags used to derive the estimate */
        std::vector<found_apriltag_t> tags;
        /** A Pose3d object storing the estimated pose */
        frc::Pose3d position;
        /** The timestamp in seconds representing and estimate of when the robot was at this position */
        double timestamp;
        /** Represents the measured error in pixels */
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