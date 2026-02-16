// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimator
// Purpose: Define localization structs + Base PositionSolver class
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include "src/utils/PCH.h"

namespace localiztaion {
    /** Represents a detected apriltag from a tframe */
    using detected_apriltag_t = struct AprilTagDetection {
        /** The unique id of tag */
        int tag_id;
        /** An array of each tag corner coordinate in pixels */
        std::array<cv::Point2d, 4> cornerCoords;
        /** The timestamp in seconds representing when this tag was viewed at this angle aka the timestamp of the timestamped_frame_t it was derived from */
        double timestampSeconds;
    };

    /** Represents a robot position estimate in 3D space (x, y, z) */
    using pose3d_estimate_t = struct Pose3dEstimate {
        /** A Pose3d object storing the estimated pose */
        frc::Pose3d position;
        /** The timestamp in seconds representing and estimate of when the robot was at this position */
        double timestamp;
        /** Represents the measured error in estimation */
        double variance;
    };

    /** Represents a class for estimating robot positions from detected tags */
    class IPositionEstimator {
        virtual ~IPositionEstimator() = default;
        /** Return 3d position estimates from detected tags */
        virtual auto estimatePosition(const std::vector<detected_apriltag_t>& detected_tags) -> std::vector<pose3d_estimate_t> = 0;
    };
}