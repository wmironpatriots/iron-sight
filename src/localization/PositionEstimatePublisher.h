// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PositionEstimatePublisher.h
// Purpose: Define the PositionEstimatePublisher class
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/utils/PCH.h"
#include "src/camera/CameraConfig.h"
#include "src/localization/PositionEstimator.h"

namespace localization {
    /** Wrapper for a set of NetworkTables Publishers used to send a PositionEstimate */
    class PositionEstimatePublisher {
        public:
            PositionEstimatePublisher(const camera::CameraConfig& config);
            /** Send the components of a new 3d PositionEstimate */
            void Publish(const localization::pose3d_estimate_t& estimate);
        private:
            std::shared_ptr<nt::NetworkTable> table_;

            nt::StructPublisher<frc::Pose3d> pose3d_publisher;
            nt::StructPublisher<frc::Pose2d> pose2d_publisher;
            nt::DoublePublisher timestamp_publisher_;
            nt::DoublePublisher variance_publisher_;

            std::mutex mutex_;
    };
}
