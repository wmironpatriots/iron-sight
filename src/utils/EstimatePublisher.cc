// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: EstimatePublisher.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/utils/EstimatePublisher.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/StructTopic.h>
#include "src/localization/PositionEstimator.h"

namespace utils {
    EstimatePublisher::EstimatePublisher(const std::string& cameraName) : mTable(kNtInstance.GetTable("iron-sight/estimates/" + cameraName)){
        mPose3dPublisher = mTable->GetStructTopic<frc::Pose3d>("Pose3d").Publish();
        mPose2dPublisher = mTable->GetStructTopic<frc::Pose2d>("Pose2d").Publish();

        mTimestampPublisher = mTable->GetDoubleTopic("TimestampSeconds").Publish();
        mVariancePublisher = mTable->GetDoubleTopic("Variance").Publish();
    }

    void EstimatePublisher::publish(const localization::pose3d_estimate_t& estimate) {
        mMutex.lock();
            mPose3dPublisher.Set(estimate.position);
            mPose2dPublisher.Set(estimate.position.ToPose2d());

            mTimestampPublisher.Set(estimate.timestamp);
            mVariancePublisher.Set(estimate.variance);
        mMutex.unlock();
    }
}
