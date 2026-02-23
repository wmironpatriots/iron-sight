add_library(utils NtUtils.cc VisionUtils.cc EstimatePublisher.cc) 
target_link_libraries(utils PUBLIC wpilibc nlohmann_json::nlohmann_json)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(utils PUBLIC pch.h)
endif()