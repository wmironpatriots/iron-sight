// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: NtUtils.h
// Purpose: Define utility methods for interacting with NetworkTables
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#pragma once
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/Topic.h>
#include <string>

namespace utils {
    /** The default NetworkTables Instance */
    const nt::NetworkTableInstance kNtInstance = nt::NetworkTableInstance::GetDefault(); 

    /** The team number used for all connections */
    const int kTeamNumber = 6423;

    /** The directory to store outputs on NetworkTables */
    const std::string kPrefix = "/iron-sight";

    /**
     * Start NetworkTables Connection
     * 
     * Create a NT4 client named 'iron-sight' and attempt
     * to connect it 
     */
    void StartNetworkTables();
}