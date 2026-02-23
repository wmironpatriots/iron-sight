// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: NtUtils.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/utils/NtUtils.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTable.h>
#include <networktables/StructTopic.h>
#include <frc/DataLogManager.h>
#include <ntcore_cpp.h>
#include <chrono>
#include <memory>
#include <thread>

namespace utils {
    void StartNetworkTables() {
        nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();

        instance.StopClient();
        instance.StopLocal();

        instance.StartClient4("iron-sight");
        instance.SetServerTeam(kTeamNumber);
        instance.StartDSClient();

        // frc::DataLogManager::Start(kPrefix + "/logs");

        printf("Waiting for Connection to %d\n", kTeamNumber);
        while (!kNtInstance.IsConnected()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        printf("Connected!");
    }
}
