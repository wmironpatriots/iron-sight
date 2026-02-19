// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// File: Yolo.h
// TODO purpose
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#pragma once

#include <string>
#include <vector>
#include "src/utils/PCH.h"
#include "opencv4/opencv2/dnn/dnn.hpp"

namespace yolo {
  class Yolo {
    public:
      Yolo(const std::string& model_path, bool swap_rb, bool verbose = false);
      ~Yolo() = default;

      auto RunModel(const cv::Mat& frame) -> std::vector<float>;

      auto Postprocess(const int original_height, const int original_width,
                      const std::vector<float>& results,
                      std::vector<cv::Rect>& bboxes,
                      std::vector<float>& confidences,
                      std::vector<int>& class_ids) -> std::vector<float>;

      static auto GetObjectAngle(double object_position, double fov,
                                int image_width = 640) -> double;

      static void DrawDetections(cv::Mat& img, const std::vector<cv::Rect>& boxes,
                                const std::vector<int>& class_ids,
                                const std::vector<float>& confidences,
                                const std::vector<std::string>& class_names);

      static constexpr int TARGET_SIZE = 320;  //I have no clue if this is right

      static constexpr float CONF_THRESH = 0.25f;
      static constexpr float NMS_THRESH = 0.45f;

    private:
      auto PreprocessImage(const cv::Mat& frame) -> cv::Mat;

      static void Nms(const std::vector<cv::Rect>& boxes,
                      const std::vector<float>& scores, float score_thresh,
                      float nms_thresh, std::vector<int>& kept_indices);

      cv::dnn::Net net_;
      bool swap_rb_;
      bool verbose_;
  };
}