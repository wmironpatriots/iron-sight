// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: GeometryUtils.h
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/utils/GeometryUtils.h"
#include <frc/geometry/CoordinateSystem.h>
#include <frc/geometry/Pose3d.h>

namespace utils {
    auto OpenCvTransformToWpilibTransform(cv::Mat rvec, cv::Mat tvec) -> frc::Transform3d {
        // Convert cv tvec into Eigen vector
        Eigen::Vector3d T(
            tvec.at<double>(0), 
            tvec.at<double>(1), 
            tvec.at<double>(2));
        auto translation = frc::Translation3d(T);

        // Convert cv rvec into Eigen vector
        Eigen::Vector3d R(
            rvec.at<double>(0), 
            rvec.at<double>(1), 
            rvec.at<double>(2));
        auto rotation = frc::Rotation3d();

        if (R.norm() > 1e-6) {
            rotation = frc::Rotation3d(R, units::angle::radian_t{R.norm()});
        }

        auto cv = frc::Transform3d(translation, rotation).Inverse();
        auto wpilib = frc::CoordinateSystem::Convert(
            cv, 
            frc::CoordinateSystem::EDN(), 
            frc::CoordinateSystem::NWU());
        
        return wpilib;
    }

    auto WpilibCoordSysToOpenCvCoordSys(frc::Pose3d pose) -> frc::Pose3d {
        return frc::CoordinateSystem::Convert(pose, frc::CoordinateSystem::NWU(), frc::CoordinateSystem::EDN());
    }

    auto WpilibCoordSysToOpenCvCoordSys(frc::Transform3d transform) -> frc::Transform3d {
        return frc::CoordinateSystem::Convert(transform, frc::CoordinateSystem::NWU(), frc::CoordinateSystem::EDN());
    }

    auto OpenCvCoordSysToWpilibCoordSys(frc::Pose3d pose) -> frc::Pose3d {
        return frc::CoordinateSystem::Convert(pose, frc::CoordinateSystem::EDN(), frc::CoordinateSystem::NWU());
    }

    auto OpenCvCoordSysToWpilibCoordSys(frc::Transform3d transform) -> frc::Transform3d {
        return frc::CoordinateSystem::Convert(transform, frc::CoordinateSystem::EDN(), frc::CoordinateSystem::NWU());
    }
}
