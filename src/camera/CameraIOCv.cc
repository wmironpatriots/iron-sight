// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: CameraIOCv.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/camera/CameraConfig.h"
#include "src/camera/CameraIOCv.h"
#include <frc/Timer.h>
#include <cstdio>
#include <filesystem>
#include <opencv2/videoio.hpp>

namespace camera {
    CameraIOCv::CameraIOCv(const camera_config_t& config) {
        config_ = config;
        std::string path = std::filesystem::read_symlink(config.device_id);
        
        int camIndex = path[path.length()] - 0;
        camera_capture_ = cv::VideoCapture(camIndex, config.api_id);
        
        camera_capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.capture_width);
        camera_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.capture_height); 
        camera_capture_.set(cv::CAP_PROP_FPS, config.capture_fps);
        std::string codec = config.codec;
        camera_capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]));

        backup_img_ = cv::imread(kBackupImgPath);
        cv::resize(backup_img_, backup_img_, cv::Size(config.capture_width, config.capture_height));

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

    auto CameraIOCv::GetTimestampedFrame() -> timestamped_frame_t {
        timestamped_frame_t tframe;
        tframe.frame = GetFrame();
        tframe.timestamp_seconds = frc::Timer::GetFPGATimestamp().to<double>();

        printf("%f\n", tframe.timestamp_seconds);

        return tframe;
    }

    auto CameraIOCv::Restart() -> void {
        camera_capture_.release();
        camera_capture_ = cv::VideoCapture(config_.device_id, config_.api_id);

        camera_capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.capture_width);
        camera_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.capture_height); 
        camera_capture_.set(cv::CAP_PROP_FPS, config_.capture_fps);
        std::string codec = config_.codec;
        camera_capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(codec[0], codec[1], codec[2], codec[3]));
    }
}
