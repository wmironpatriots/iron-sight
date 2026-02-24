// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: CameraIOCv.h
// TODO Restart Function
// TODO More failsafes?
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"

namespace camera {
    CameraIOCv::CameraIOCv(const CameraConfig& config) {
        config_ = config;

        camera_capture_ = cv::VideoCapture(config.deviceId, config.apiId);

        camera_capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.captureWidth);
        camera_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.captureHeight); 
        camera_capture_.set(cv::CAP_PROP_FPS, config.captureFPS);
        std::string codec = config.codec;
        camera_capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]));

        backup_img_ = cv::imread(kBackupImgPath);
        cv::resize(backup_img_, backup_img_, cv::Size(config.captureWidth, config.captureHeight));

        if (!camera_capture_.isOpened()) {
            printf("Error ~ Failed to open camera\n");
            throw;
        }
    }

    auto CameraIOCv::GetConfig() -> camera_config_t {
        return config_;
    }

    auto CameraIOCv::GetFrame() -> cv::Mat {
        cv::Mat frame;
        camera_capture_.read(frame);

        if (frame.empty()) {
            frame = backup_img_;
        }

        return frame;
    }

    auto CameraIOCv::GetTimestampedFrame() -> TimestampedFrame {
        TimestampedFrame tframe;
        tframe.frame = GetFrame();
        tframe.timestamp = frc::Timer::GetFPGATimestamp();

        return tframe;
    }

    auto CameraIOCv::Restart() -> void {
        // TODO
    }
}
