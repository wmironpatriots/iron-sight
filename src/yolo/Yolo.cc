// Copyright (c) 2026 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/wmironpatriots
//
// Header: yolo.h
//
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

#include "Yolo.h"

namespace yolo {

static auto Letterbox(const cv::Mat& img, int target_size, float& out_scale,
                         int& out_pad_left, int& out_pad_top, bool swap_rb) -> cv::Mat{
  cv::Mat src = img;

  if (swap_rb) {
    cv::cvtColor(img, src, cv::COLOR_BGR2RGB);
  }

  const int orig_h = src.rows;
  const int orig_w = src.cols;

  out_scale =
      std::min(target_size / (float)orig_h, target_size / (float)orig_w);
  const int new_w = (int)std::round(orig_w * out_scale);
  const int new_h = (int)std::round(orig_h * out_scale);

  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  const int dw = target_size - new_w;
  const int dh = target_size - new_h;
  out_pad_left = (int)std::round(dw / 2.0 - 0.1);
  out_pad_top = (int)std::round(dh / 2.0 - 0.1);
  const int pad_right = (int)std::round(dw / 2.0 + 0.1);
  const int pad_bottom = (int)std::round(dh / 2.0 + 0.1);
  cv::Mat padded;
  cv::copyMakeBorder(resized, padded, out_pad_top, pad_bottom, out_pad_left,
                     pad_right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

  return padded;
}

Yolo::Yolo(const std::string& model_path, bool swap_rb, bool verbose)
    : swap_rb_(swap_rb), verbose_(verbose) {
  net_ = cv::dnn::readNetFromONNX(model_path);
  if (net_.empty()) {
    throw std::runtime_error("Failed to load ONNX model: " + model_path);
  }

  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  if (verbose_) {
    std::cout << "[YOLO] Loaded ONNX: " << model_path << "\n";
  }
}

auto Yolo::PreprocessImage(const cv::Mat& frame) -> cv::Mat {
  float scale = 1.0f;
  int pad_left = 0;
  int pad_top = 0;

  cv::Mat padded =
      Letterbox(frame, TARGET_SIZE, scale, pad_left, pad_top, swap_rb_);
  cv::Mat blob = cv::dnn::blobFromImage(padded, 1.0 / 255.0,
                                        cv::Size(TARGET_SIZE, TARGET_SIZE),
                                        cv::Scalar(), false, false, CV_32F);
  std::cout << "blob dims=" << blob.dims << " shape=[";
  for (int i = 0; i < blob.dims; i++) std::cout << blob.size[i] << (i+1<blob.dims?", ":"");
  std::cout << "] type=" << blob.type() << "\n";
  return blob;
}

auto Yolo::RunModel(const cv::Mat& frame) -> cv::Mat {
  cv::Mat blob = PreprocessImage(frame);
  net_.setInput(blob);

  cv::Mat out = net_.forward();
  std::vector<float> results;
  results.assign((float*)out.datastart, (float*)out.dataend);

  if (verbose_) {
    std::cout << "[YOLO] Output dims: " << out.dims << " [";
    for (int i = 0; i < out.dims; i++) {
      std::cout << out.size[i] << (i + 1 < out.dims ? ", " : "");
    }
    std::cout << "], type=" << out.type() << "\n";
  }

  // Ensure float output for decoding
  if (out.type() != CV_32F) {
    cv::Mat out32f;
    out.convertTo(out32f, CV_32F);
    return out32f;
  }

  return out;
}

void Yolo::Nms(const std::vector<cv::Rect>& boxes,
               const std::vector<float>& scores, float score_thresh,
               float nms_thresh, std::vector<int>& kept_indices) {
  kept_indices.clear();
  if (boxes.empty())
    return;
  cv::dnn::NMSBoxes(boxes, scores, score_thresh, nms_thresh, kept_indices);
}

void yolo::Yolo::Postprocess(int original_height, int original_width,
                             const cv::Mat& out,
                             std::vector<cv::Rect>& bboxes,
                             std::vector<float>& confidences,
                             std::vector<int>& class_ids) {
  bboxes.clear();
  confidences.clear();
  class_ids.clear();
  if (out.empty()) return;

  
  cv::Mat out32 = out;
  if (out32.type() != CV_32F) out32.convertTo(out32, CV_32F);
  if (!out32.isContinuous()) out32 = out32.clone();

  
  const float gain =
      std::min(TARGET_SIZE / (float)original_height, TARGET_SIZE / (float)original_width);
  const int new_w = (int)std::round(original_width * gain);
  const int new_h = (int)std::round(original_height * gain);
  const int dw = TARGET_SIZE - new_w;
  const int dh = TARGET_SIZE - new_h;
  const auto pad_left = (float)std::round(dw / 2.0 - 0.1);
  const auto pad_top  = (float)std::round(dh / 2.0 - 0.1);

  
  cv::Mat feat_major;  // [C, N]
  int C = 0;
  int N = 0;

  if (out32.dims == 3) {
    if (out32.size[0] != 1) return;

    const int d1 = out32.size[1];
    const int d2 = out32.size[2];

    const bool looks_like_C_N = (d1 < 512 && d2 > 512); // [1, C, N]
    const bool looks_like_N_C = (d2 < 512 && d1 > 512); // [1, N, C]

    if (looks_like_C_N) {
      
      feat_major = cv::Mat(d1, d2, CV_32F, (void*)out32.ptr<float>());
      C = d1;
      N = d2;
    } else if (looks_like_N_C) {
      
      cv::Mat tmp(d1, d2, CV_32F, (void*)out32.ptr<float>()); // [N, C]
      cv::transpose(tmp, feat_major);                          // [C, N]
      feat_major = feat_major.clone();                         // contiguous owning buffer
      C = feat_major.rows;
      N = feat_major.cols;
    } else {
      return;
    }
  } else if (out32.dims == 2) {
    
    const int r = out32.rows;
    const int c = out32.cols;
    if (r < c) {
      feat_major = out32; // [C, N]
      C = r; N = c;
    } else {
      cv::transpose(out32, feat_major); // [C, N]
      feat_major = feat_major.clone();
      C = feat_major.rows;
      N = feat_major.cols;
    }
  } else {
    return;
  }

  if (C < 5 || N <= 0) return;
  const int num_classes = C - 4;

  const float* ptr = feat_major.ptr<float>();

  std::vector<cv::Rect> raw_boxes;
  std::vector<float> raw_scores;
  std::vector<int> raw_class_ids;
  raw_boxes.reserve(N);
  raw_scores.reserve(N);
  raw_class_ids.reserve(N);

  
  for (int d = 0; d < N; ++d) {
    float cx = ptr[0 * N + d];
    float cy = ptr[1 * N + d];
    float w  = ptr[2 * N + d];
    float h  = ptr[3 * N + d];

    
    if (cx <= 1.5f && cy <= 1.5f && w <= 1.5f && h <= 1.5f) {
      cx *= TARGET_SIZE; cy *= TARGET_SIZE;
      w  *= TARGET_SIZE; h  *= TARGET_SIZE;
    }

    
    int best_cid = -1;
    float best = -1e9f;
    for (int c = 0; c < num_classes; ++c) {
      const float s = ptr[(4 + c) * N + d];
      if (s > best) {
        best = s;
        best_cid = c;
      }
    }

    if (best_cid < 0 || best < CONF_THRESH) continue;

    // cxcywh -> top-left in letterboxed coords
    float left = cx - 0.5f * w;
    float top  = cy - 0.5f * h;

    // Undo letterbox back to original image coords
    float x = (left - pad_left) / gain;
    float y = (top  - pad_top)  / gain;
    float bw = w / gain;
    float bh = h / gain;

    // Clamp to image bounds
    x  = std::max(0.0f, std::min(x,  (float)original_width  - 1.0f));
    y  = std::max(0.0f, std::min(y,  (float)original_height - 1.0f));
    bw = std::max(0.0f, std::min(bw, (float)original_width  - x));
    bh = std::max(0.0f, std::min(bh, (float)original_height - y));

    if (bw <= 2.0f || bh <= 2.0f) continue;

    raw_boxes.emplace_back((int)std::round(x),
                           (int)std::round(y),
                           (int)std::round(bw),
                           (int)std::round(bh));
    raw_scores.emplace_back(best);
    raw_class_ids.emplace_back(best_cid);
  }

  std::vector<int> kept;
  cv::dnn::NMSBoxes(raw_boxes, raw_scores, CONF_THRESH, NMS_THRESH, kept);

  bboxes.reserve(kept.size());
  confidences.reserve(kept.size());
  class_ids.reserve(kept.size());
  for (int idx : kept) {
    bboxes.push_back(raw_boxes[idx]);
    confidences.push_back(raw_scores[idx]);
    class_ids.push_back(raw_class_ids[idx]);
  }
}

auto Yolo::GetObjectAngle(double object_position, double fov,
                          int image_width) -> double {
  const double focal_length = image_width / std::tan(fov / 2.0);
  return std::atan2(object_position - (image_width / 2.0), focal_length);
}

void Yolo::DrawDetections(cv::Mat& img, const std::vector<cv::Rect>& boxes,
                          const std::vector<int>& class_ids,
                          const std::vector<float>& confidences,
                          const std::vector<std::string>& class_names) {
  for (size_t i = 0; i < boxes.size(); i++) {
    cv::rectangle(img, boxes[i], cv::Scalar(0, 255, 0), 2);

    const int cid = class_ids[i];
    const std::string name =
        (cid >= 0 && std::abs(cid) < (int)class_names.size()) ? class_names[cid] : "obj";

    const std::string label = name + " " + cv::format("%.2f", confidences[i]);

    int baseline = 0;
    const cv::Size label_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

    cv::rectangle(
        img,
        cv::Point(boxes[i].x,
                  std::max(0, boxes[i].y - label_size.height - baseline)),
        cv::Point(boxes[i].x + label_size.width, boxes[i].y),
        cv::Scalar(0, 255, 0), cv::FILLED);

    cv::putText(img, label, cv::Point(boxes[i].x, boxes[i].y - baseline),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }
}

}  // namespace yolo