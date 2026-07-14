// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: NtUtils.h
// Purpose: Define utility functions for interacting with NetworkTables
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include "src/utils/PCH.h"

namespace utils {
    /** Default NetworkTables Instance */
    const nt::NetworkTableInstance kNtInstance = nt::NetworkTableInstance::GetDefault(); 

    /** Team Number used for Server Address */
    const int kTeamNumber = 6423;

    /** The directory to store outputs on NetworkTables */
    /** Topic to store all telemetry on NetworkTables */
    const std::string kPrefix = "/iron-sight";

    /**
     * Start NetworkTables Connection
     * 
     * Create a NT4 client named 'iron-sight' and attempt
     * to connect it 
     *
     * If running in simulation mode, set isSimulated to true
     */
    void StartNetworkTables(bool isSimulated);
}