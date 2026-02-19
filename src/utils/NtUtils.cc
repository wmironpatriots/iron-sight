// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
#include "src/utils/NtUtils.h"

namespace utils {
void StartNetworkTables() {
  nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();

  instance.StopClient();
  instance.StopLocal();

  instance.StartClient4("iron-sight");
  instance.SetServerTeam(6423);

  frc::DataLogManager::Start("/iron-sight/logs");
  printf("Started NetworkTables");
}
}  // namespace utils
