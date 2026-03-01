// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: NtUtils.h
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "src/utils/NtUtils.h"

namespace utils {
    void StartNetworkTables(bool isSimulated) {
        nt::NetworkTableInstance instance = kNtInstance;

        instance.StopClient();
        instance.StopLocal();

        instance.StartClient4("iron-sight");
        instance.SetServerTeam(kTeamNumber);
        if (isSimulated) instance.SetServer("192.168.69.45", nt::NetworkTableInstance::kDefaultPort4);
        instance.StartDSClient();

        // frc::DataLogManager::Start(kPrefix + "/logs");

        printf("Waiting for Connection to %d\n", kTeamNumber);
        while (!kNtInstance.IsConnected()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        printf("Connected!");
    }
}
