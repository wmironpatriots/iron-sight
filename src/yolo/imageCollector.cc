#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/Camera.h"
#include "src/camera/CameraCv.h"
#include "src/utils/PCH.h"

auto main() -> int {
  std::string img_dir = "iron-sight/logs/collected_imgs";
  std::filesystem::create_directories(img_dir);
  camera::CameraConfig config{};
  config.deviceId = 0;
  config.apiId = cv::CAP_ANY;

  camera::CameraCv cam(config);
  while (true) {
    camera::TimestampedFrame tframe = cam.getTimestampedFrame();

    if (!tframe.frame.empty()) {
      const std::string path =
          img_dir + std::to_string(tframe.timestamp.value()) + ".png";
      cv::imwrite(path, tframe.frame);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }
}