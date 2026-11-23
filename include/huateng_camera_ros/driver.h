// Camera driver core without ROS: opens the camera with the Huateng (MindVision MVSDK) SDK, grabs images,
// computes time stamps and hands finished frames to the ROS layer (src/ros1_node.cpp, src/ros2_node.cpp).
//
// Threads:
//  - grab thread: fetches every image from the SDK, notes the receive time and the SDK frame head (uiTimeStamp,
//    uiExpTime), computes the time stamp (trigger_clock.h) and runs the ISP. It never blocks on ROS.
//  - publish thread: calls Callbacks::publish (mono/JPEG conversion and publishing in the ROS layer).
//  - Conversions only for topics that have subscribers (Callbacks::want_image).
#pragma once

#include <huateng_camera_ros/config.h>

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace huateng_camera_ros {

struct Frame {
  int64_t stamp_ns = 0;  // host time or LiDAR/MCU time (Config::time_source), ns since epoch
  cv::Mat buffer;        // memory of the ISP result (keeps image alive)
  cv::Mat image;         // 8UC3, RGB or BGR (Frame::is_rgb); empty if nobody wants an image
  bool is_rgb = true;
};

// Grayscale copy into dst (h x w, CV_8UC1, may point into a message buffer)
void toMono(const Frame &f, cv::Mat &dst);
// Color image as packed 8UC3 bytes (RGB or BGR as produced by the ISP)
const uint8_t *colorData(const Frame &f);
// JPEG (always color, BGR order for the encoder)
bool toJpeg(const Frame &f, int quality, std::vector<uint8_t> &out);

class Driver {
 public:
  struct Callbacks {
    std::function<bool()> want_image;             // any subscriber for raw or compressed image?
    std::function<void(const Frame &)> publish;   // called in the publish thread (also for frames without image)
    LogFn log;
  };

  Driver(const Config &cfg, Callbacks cb);
  ~Driver();
  Driver(const Driver &) = delete;
  Driver &operator=(const Driver &) = delete;

  bool start();  // open camera, start threads; false on error (logged)
  void stop();

  // One line with statistics since the last call (call e.g. every 10 s)
  std::string statsLine();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace huateng_camera_ros
