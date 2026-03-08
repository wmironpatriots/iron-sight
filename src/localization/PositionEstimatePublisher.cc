// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: PositionEstimatePublisher.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/localization/PositionEstimatePublisher.h"
#include "src/utils/NtUtils.h"

namespace localization {
    PositionEstimatePublisher::PositionEstimatePublisher(const camera::CameraConfig& config) : table_(utils::kNtInstance.GetTable(utils::kPrefix + "/estimates/" + config.nickname)) {
        pose3d_publisher = table_->GetStructTopic<frc::Pose3d>("Pose3d").Publish();
        pose2d_publisher = table_->GetStructTopic<frc::Pose2d>("Pose2d").Publish();

        timestamp_publisher_ = table_->GetDoubleTopic("TimestampSeconds").Publish();
        variance_publisher_ = table_->GetDoubleTopic("Variance").Publish();
        tags_used_publisher_ = table_->GetIntegerTopic("tagsUsed").Publish();
    }

    void PositionEstimatePublisher::Publish(const localization::pose3d_estimate_t& estimate) {
        mutex_.lock();

            pose3d_publisher.Set(estimate.position);
            pose2d_publisher.Set(estimate.position.ToPose2d());

            timestamp_publisher_.Set(estimate.timestamp);
            variance_publisher_.Set(estimate.variance);
            tags_used_publisher_.Set(estimate.tagsUsed);
        mutex_.unlock();
    }
}
