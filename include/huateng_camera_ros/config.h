// Driver configuration, read from a YAML file with cv::FileStorage (same file for ROS 1 and ROS 2).
// See config/camera.yaml for all keys and their meaning.
#pragma once

#include <functional>
#include <string>

namespace huateng_camera_ros {

enum class LogLevel { Debug, Info, Warn, Error, Fatal };
using LogFn = std::function<void(LogLevel, const std::string &)>;

enum class TimeSource {
  Host,   // stamp with the host clock (normal camera driver)
  Lidar,  // stamp in LiDAR/MCU time from timeshare (hardware sync, see README "LiDAR time sync")
};

struct Config {
  // Names and paths
  std::string camera_name = "camera";
  std::string frame_id = "camera_frame";
  std::string topic_raw = "camera/image_raw";
  std::string topic_compressed = "camera/image/compressed";
  std::string topic_camera_info = "camera/camera_info";
  std::string calibration_url;   // empty = camera_info_manager default (~/.ros/camera_info/<camera_name>.yaml)
  std::string serial;            // empty = first camera found
  std::string timeshare_path;    // empty = $HOME/timeshare (default of livox_ros_driver in LIV_handhold)

  // Camera
  bool exposure_auto = true;
  int exposure_target = 120;
  double exposure_time_us = 10000.0;  // used only with ExposureAuto Off
  double ae_exposure_max_us = -1.0;   // AE: longest exposure; -1 = keep camera setting
  double ae_gain_max = -1.0;          // AE: highest analog gain (factor); -1 = keep camera setting
  int ae_restart_every = 1;           // AE with trigger: re-enable AE every N images (0 = off), see camera.yaml
  int trigger_mode = 0;               // 0 = continuous, 1 = software, 2 = hardware
  std::string trigger_signal;         // empty = keep camera setting, else "RisingEdge" / "FallingEdge"
  bool wb_auto = true;
  int orientation = 0;                // clockwise: 0, 1 = 90 deg, 2 = 180 deg, 3 = 270 deg
  bool isp_bgr = false;               // ISP output BGR8 instead of RGB8
  int gamma = -1;                     // -1 = keep camera setting
  int frame_speed = -1;               // -1 = highest level, else index (0 low, 1 normal, 2 high, ...)

  // Output
  bool raw_mono = false;  // raw image as mono8 instead of color
  int jpeg_quality = 90;
  int queue_size = 3;

  // Time stamps
  TimeSource time_source = TimeSource::Host;
  double trigger_period = 0.1;      // s, hardware trigger period
  double transfer_latency = 0.012;  // s, GigE transfer of one image (rough estimate)
  bool stamp_mid_exposure = false;
  bool snap_to_grid = true;         // LiDAR mode: round to the trigger grid
  double trigger_phase = 0.0;       // s, trigger edge relative to the PPS/LiDAR grid
  bool lidar_fallback_host = false; // LiDAR mode without LiDAR time: stamp with host time instead of dropping

  // Diagnostics: log raw values of the first N images
  int log_frames = 0;
};

// Missing keys keep their defaults. Returns false if the file could not be opened (defaults are used then).
bool loadConfig(const std::string &path, Config &cfg, const LogFn &log);

// "host" / "lidar" (case-insensitive); returns false for anything else
bool parseTimeSource(const std::string &s, TimeSource &out);

const char *toString(TimeSource t);

}  // namespace huateng_camera_ros
