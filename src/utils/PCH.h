// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: PCH.h
// Purpose: Compile commonly use headers once for faster compile times
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

// C++ Standard Library
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// OpenCV
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>

// AprilTag
#include "apriltag/frc/apriltag/AprilTagFieldLayout.h"
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

// Eigen
#include <Eigen/Core>

// Sleipnir
#include <sleipnir/optimization/problem.hpp>

// FRC/WPILib
#include <wpilibc/frc/Timer.h>
#include <wpilibc/frc/DataLogManager.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <wpimath/frc/geometry/Pose3d.h>
#include <wpimath/frc/geometry/Transform3d.h>
#include <wpimath/frc/geometry/Translation3d.h>
#include <wpimath/frc/geometry/Rotation3d.h>

// NetworkTables
#include <networktables/NetworkTable.h>
#include <networktables/StructTopic.h>
#include <networktables/DoubleTopic.h>
