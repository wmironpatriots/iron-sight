// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: ModuleConstants.h
// Purpose: Define yolo config structs
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include <string>
#include <vector>
#include "src/utils/PCH.h"

namespace yolo {
/** Represents a configuration for a module */
using module_config_t = struct ModuleConfig {
  /** The path to the model */
  const std::string path;
  /** The catagories the model contains */
  const std::vector<std::string> classes;
  /** The width of the images in the model */
  const int width;
  /** The height of the images in the model */
  const int height;
  /** True if model uses colored images */
  const bool swap_rb;

  ModuleConfig(std::string p, std::vector<std::string> cls, int w, int h,
               bool swap)
      : path(std::move(p)),
        classes(std::move(cls)),
        width(w),
        height(h),
        swap_rb(swap) {}
};

inline const module_config_t kAlphaModel{"models/best_opencv.onnx",
                                         std::vector<std::string>{"fuel"}, 640,
                                         640, true};
}  // namespace yolo