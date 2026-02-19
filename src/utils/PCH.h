// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PCH.h
// Purpose: Compile all headers at once for faster compile times
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

// C++ Standard Library
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// OpenCV
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/videoio.hpp>

// Eigen
#include <Eigen/Core>

// Sleipnir
#include <sleipnir/optimization/problem.hpp>

// FRC/WPILib
#include <ntcore/networktables/NetworkTableInstance.h>
#include <wpilibc/frc/DataLogManager.h>
#include <wpilibc/frc/Timer.h>

// JSON
#include <nlohmann/json.hpp>
