// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CameraIOCv.h
// Purpose: Define CameraIO with a OpenCV backend
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/camera/Camera.h"
#include "src/camera/CameraConfig.h"

namespace camera {
    /** Image to use in-case of undesired behavior */
    const std::string kBackupImgPath = "./resources/bessie.png";

    /** CameraIO extension using OpenCV as backend */
    class CameraIOCv : public CameraIO {
        public:
            CameraIOCv(const camera_config_t& config);            
            auto GetConfig() -> camera_config_t override;
            /** Return raw OpenCV Matrix */
            auto GetFrame() -> cv::Mat;
            auto GetTimestampedFrame() -> timestamped_frame_t override;
            auto Restart() -> void override;
        private:
            CameraConfig config_;
            cv::VideoCapture camera_capture_;
            cv::Mat backup_img_;
    };

}
