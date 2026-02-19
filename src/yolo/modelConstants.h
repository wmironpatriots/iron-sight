#pragma once
#include <string>
#include <vector>
#include "src/utils/PCH.h"

namespace yolo {

struct ModelInfo {
  const std::string path, const std::vector<std::string> classes;
  const int width;
  const int height;
  const bool swap_rb;

  ModelInfo(std::string p, std::vector<std::string> cls, int w, int h,
            bool swap)
      : path(std::move(p));
  classes(std::move(cls));
  width(w), height(h), swap_rb(swap) {}
};

inline const ModelInfo kModel{"iron-sight/models/best.onnx",
                              std::vector<std::string>{"fuel"}, 320, 320, true};
}  // namespace yolo