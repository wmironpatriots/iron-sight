// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: CalibrationUtils.h
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/utils/CalibrationUtils.h"
#include <opencv2/core/mat.hpp>

namespace utils {
    auto CameraMatrixFromIntrinsics(double fx, double fy, double cx, double cy) -> cv::Mat {
        cv::Mat mat = (cv::Mat_<double>(3, 3) <<
            fx, 0,  cx,
            0,  fy, cy,
            0,  0,  1);

        return mat;
    }

    auto DistortionCoefficentsFromIntrinsics(double p1, double p2, double k1, double k2, double k3) -> cv::Mat {
        cv::Mat mat = (cv::Mat_<double>(1, 5) <<
            k1, k2, p1, p2, k3);

        return mat;
    }
}
