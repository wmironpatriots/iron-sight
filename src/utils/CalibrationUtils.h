// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: CalibrationUtils.h
// Purpose: Define utility functions for utilizing & modifying camera calibrations
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/utils/PCH.h"

namespace utils {
    /**
     * Generate a camera matrix from intrinsic values
     *
     * @see https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
     */
    auto CameraMatrixFromIntrinsics(double fx, double fy, double cx, double cy) -> cv::Mat;

    /**
     * Generate a distortion coefficents matrix from intrinsic values
     *
     * @see https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
     */
    auto DistortionCoefficentsFromIntrinsics(double p1, double p2, double k1, double k2, double k3) -> cv::Mat;
}
