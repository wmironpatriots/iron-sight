// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: EstimatePublisher.h
// Purpose: Define the estimate publisher class
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once

#include <networktables/NetworkTable.h>
#include <memory>
#include <mutex>
#include "src/utils/NtUtils.h"

// TODO docs
namespace utils {
    class EstimatePublisher {
        public:
            EstimatePublisher(const std::string& cameraName);
            void publish(const localization::pose3d_estimate_t& estimate);
        private:
            std::shared_ptr<nt::NetworkTable> mTable;

            nt::StructPublisher<frc::Pose3d> mPose3dPublisher;
            nt::StructPublisher<frc::Pose2d> mPose2dPublisher;
            nt::DoublePublisher mTimestampPublisher;
            nt::DoublePublisher mVariancePublisher;

            std::mutex mMutex;
    };
}