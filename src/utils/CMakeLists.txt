add_library(utils NtUtils.cc CalibrationUtils.cc GeometryUtils.cc) 
target_link_libraries(utils PUBLIC wpilibc)

if(!ENABLE_CLANG_TIDY)
  target_precompile_headers(utils PUBLIC pch.h)
endif()