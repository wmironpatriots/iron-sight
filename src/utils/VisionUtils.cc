// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/utils/VisionUtils.h"
#include <frc/geometry/Rotation3d.h>

namespace utils {
    auto ConvertOpencvRvecTvecToWpiLibTransform(cv::Mat rvec, cv::Mat tvec) -> frc::Transform3d {
        Eigen::Vector3d T(
            tvec.at<double>(0), 
            tvec.at<double>(1), 
            tvec.at<double>(2));
        auto translation = frc::Translation3d(T);

        Eigen::Vector3d R(
            rvec.at<double>(0), 
            rvec.at<double>(1), 
            rvec.at<double>(2));
        auto rotation = frc::Rotation3d();

        if (R.norm() > 1e-6) {
            rotation = frc::Rotation3d(R, units::angle::radian_t{R.norm()});
        }

        auto cvFieldToCamera = frc::Transform3d(
            translation, 
            rotation).Inverse();
        auto wpilibFieldToCamera = frc::CoordinateSystem::Convert(
            cvFieldToCamera, 
            frc::CoordinateSystem::EDN(), 
            frc::CoordinateSystem::NWU());
        
        return wpilibFieldToCamera;
    }

    auto generateCameraMatrix(const camera::CameraConfig& config) -> cv::Mat {
        auto intrinsics = config.intrinsicsCalibration;
        cv::Mat matrix = (cv::Mat_<double>(3, 3) <<
            intrinsics.fx, 0,             intrinsics.cx,
            0,             intrinsics.fy, intrinsics.cy,
            0,             0,             1);

        return matrix;
    }

    auto generateDistCoeffs(const camera::CameraConfig& config) -> cv::Mat {
        auto intrinsics = config.intrinsicsCalibration;
        cv::Mat matrix = (cv::Mat_<double>(1, 5) <<
            intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2, intrinsics.k3);

        return matrix;
    }
}