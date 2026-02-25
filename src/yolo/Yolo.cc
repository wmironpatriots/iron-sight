#include "Yolo.h"
#include <opencv2/core/hal/interface.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "src/utils/PCH.h"

namespace yolo {

static cv::Mat Letterbox(const cv::Mat& img, int target_size, float& out_scale,
                         int& out_pad_left, int& out_pad_top, bool swap_rb) {
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

void Yolo::Postprocess(int original_height, int original_width,
                       const cv::Mat& out, std::vector<cv::Rect>& bboxes,
                       std::vector<float>& confidences,
                       std::vector<int>& class_ids) {
  bboxes.clear();
  confidences.clear();
  class_ids.clear();

  if (out.empty())
    return;
  CV_Assert(out.type() == CV_32F);
  CV_Assert(out.isContinuous());
  CV_Assert(out.dims == 3);
  CV_Assert(out.size[0] == 1);
  CV_Assert(out.size[1] == 5);

  const int N = out.size[2];
  const float* data = out.ptr<float>();

  const float scale = std::min(TARGET_SIZE / (float)original_height,
                               TARGET_SIZE / (float)original_width);

  const int new_w = (int)std::round(original_width * scale);
  const int new_h = (int)std::round(original_height * scale);

  const float pad_left = (TARGET_SIZE - new_w) / 2.0f;
  const float pad_top = (TARGET_SIZE - new_h) / 2.0f;

  const float* cx_ptr = data + 0 * N;
  const float* cy_ptr = data + 1 * N;
  const float* w_ptr = data + 2 * N;
  const float* h_ptr = data + 3 * N;
  const float* s_ptr = data + 4 * N;

  std::vector<cv::Rect> raw_boxes;
  std::vector<float> raw_scores;
  raw_boxes.reserve(N);
  raw_scores.reserve(N);

  for (int i = 0; i < N; i++) {
    const float score = s_ptr[i];
    if (score < CONF_THRESH)
      continue;

    float x1 = cx_ptr[i] - 0.5f * w_ptr[i];
    float y1 = cy_ptr[i] - 0.5f * h_ptr[i];
    float x2 = cx_ptr[i] + 0.5f * w_ptr[i];
    float y2 = cy_ptr[i] + 0.5f * h_ptr[i];

    x1 = (x1 - pad_left) / scale;
    y1 = (y1 - pad_top) / scale;
    x2 = (x2 - pad_left) / scale;
    y2 = (y2 - pad_top) / scale;

    x1 = std::max(0.0f, std::min(x1, (float)original_width));
    y1 = std::max(0.0f, std::min(y1, (float)original_height));
    x2 = std::max(0.0f, std::min(x2, (float)original_width));
    y2 = std::max(0.0f, std::min(y2, (float)original_height));

    const float bw = x2 - x1;
    const float bh = y2 - y1;
    if (bw <= 1.0f || bh <= 1.0f)
      continue;

    raw_boxes.emplace_back((int)x1, (int)y1, (int)bw, (int)bh);
    raw_scores.emplace_back(score);
  }

  std::vector<int> kept;
  Nms(raw_boxes, raw_scores, CONF_THRESH, NMS_THRESH, kept);

  bboxes.reserve(kept.size());
  confidences.reserve(kept.size());
  class_ids.reserve(kept.size());

  for (int idx : kept) {
    bboxes.push_back(raw_boxes[idx]);
    confidences.push_back(raw_scores[idx]);
    class_ids.push_back(0);  // one class: fuel
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
        (cid >= 0 && cid < (int)class_names.size()) ? class_names[cid] : "obj";

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