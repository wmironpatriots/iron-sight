// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header CameraStream.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/camera/CameraStream.h"
#include <fmt/base.h>
#include <cstdio>

namespace camera {
    CameraStream::CameraStream(std::unique_ptr<CameraIO> io) : io_(std::move(io)) {
        timestamped_frame_ = io_->GetTimestampedFrame();

        thread_ = std::thread([this] () -> void {
            TimestampedFrame tframe;
            tframe = io_->GetTimestampedFrame();

            mutex_.lock();
            timestamped_frame_ = tframe;
            mutex_.unlock();
        });
    };

    auto CameraStream::getTimestampedFrame() -> TimestampedFrame {
        mutex_.lock();
        TimestampedFrame tframe = timestamped_frame_;
        mutex_.unlock();

        auto currentTimestamp = frc::Timer::GetFPGATimestamp();

        if (currentTimestamp.to<double>() - tframe.timestamp.to<double>() > 0.5) {
            const auto nickname = io_->GetConfig().nickname;
            std::printf("%s has not updated recently; Attempting to restarting Camera!\n", nickname.c_str());

            mutex_.lock();
            io_->Restart();
            mutex_.unlock();
        }

        return tframe;
    };
}
