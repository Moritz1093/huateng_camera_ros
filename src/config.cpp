#include <huateng_camera_ros/config.h>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace huateng_camera_ros {
namespace {

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

void readString(const cv::FileStorage &fs, const char *key, std::string &out) {
  const cv::FileNode n = fs[key];
  if (!n.empty() && n.isString()) out = static_cast<std::string>(n);
}

void readInt(const cv::FileStorage &fs, const char *key, int &out) {
  const cv::FileNode n = fs[key];
  if (!n.empty() && n.isInt()) out = static_cast<int>(n);
}

void readDouble(const cv::FileStorage &fs, const char *key, double &out) {
  const cv::FileNode n = fs[key];
  if (!n.empty() && (n.isReal() || n.isInt())) out = static_cast<double>(n);
}

void readBool(const cv::FileStorage &fs, const char *key, bool &out) {
  const cv::FileNode n = fs[key];
  if (n.empty()) return;
  if (n.isInt()) {
    out = static_cast<int>(n) != 0;
  } else if (n.isString()) {
    const std::string s = lower(static_cast<std::string>(n));
    out = (s == "true" || s == "on" || s == "yes" || s == "1");
  }
}

}  // namespace

bool parseTimeSource(const std::string &s, TimeSource &out) {
  const std::string l = lower(s);
  if (l == "host") {
    out = TimeSource::Host;
  } else if (l == "lidar") {
    out = TimeSource::Lidar;
  } else {
    return false;
  }
  return true;
}

const char *toString(TimeSource t) { return t == TimeSource::Lidar ? "LiDAR" : "Host"; }

bool loadConfig(const std::string &path, Config &c, const LogFn &log) {
  bool ok = false;
  cv::FileStorage fs;
  try {
    ok = fs.open(path, cv::FileStorage::READ) && fs.isOpened();
  } catch (const cv::Exception &e) {
    log(LogLevel::Warn, "Config " + path + " not readable (" + e.what() + "), using defaults");
  }
  if (ok) {
    readString(fs, "CameraName", c.camera_name);
    readString(fs, "FrameId", c.frame_id);
    readString(fs, "TopicNameRaw", c.topic_raw);
    readString(fs, "TopicNameCompressed", c.topic_compressed);
    readString(fs, "TopicNameCameraInfo", c.topic_camera_info);
    readString(fs, "CalibrationPath", c.calibration_url);
    readString(fs, "SerialNumber", c.serial);
    readString(fs, "TimeStampPath", c.timeshare_path);

    readBool(fs, "ExposureAuto", c.exposure_auto);
    readInt(fs, "ExposureTarget", c.exposure_target);
    readDouble(fs, "ExposureTimeUs", c.exposure_time_us);
    std::string trig;
    readString(fs, "TriggerMode", trig);
    trig = lower(trig);
    if (trig == "continuous") c.trigger_mode = 0;
    else if (trig == "software") c.trigger_mode = 1;
    else if (trig == "hardware") c.trigger_mode = 2;
    else if (!trig.empty()) log(LogLevel::Warn, "Unknown TriggerMode '" + trig + "', keeping default");
    readString(fs, "TriggerSignal", c.trigger_signal);
    readBool(fs, "WhiteBalanceAuto", c.wb_auto);
    readInt(fs, "Orientation", c.orientation);
    std::string isp;
    readString(fs, "ISPFormat", isp);
    if (!isp.empty()) c.isp_bgr = (isp == "CAMERA_MEDIA_TYPE_BGR8" || lower(isp) == "bgr8");
    readInt(fs, "Gamma", c.gamma);
    readInt(fs, "FrameSpeed", c.frame_speed);

    std::string raw_enc;
    readString(fs, "RawEncoding", raw_enc);
    if (!raw_enc.empty()) c.raw_mono = lower(raw_enc) == "mono8";
    readInt(fs, "JpegQuality", c.jpeg_quality);
    readInt(fs, "QueueSize", c.queue_size);

    std::string ts;
    readString(fs, "TimeSource", ts);
    if (!ts.empty() && !parseTimeSource(ts, c.time_source)) {
      log(LogLevel::Warn, "Unknown TimeSource '" + ts + "' (Host / LiDAR), keeping " + toString(c.time_source));
    }
    readDouble(fs, "TriggerPeriodSec", c.trigger_period);
    readDouble(fs, "TransferLatencySec", c.transfer_latency);
    readBool(fs, "StampAtMidExposure", c.stamp_mid_exposure);
    readBool(fs, "SnapToTriggerGrid", c.snap_to_grid);
    readDouble(fs, "TriggerPhaseSec", c.trigger_phase);
    readBool(fs, "LidarFallbackHostTime", c.lidar_fallback_host);
    log(LogLevel::Info, "Config loaded: " + path);
  } else {
    log(LogLevel::Warn, "Config " + path + " not found, using defaults");
  }

  if (c.timeshare_path.empty()) {
    const char *home = std::getenv("HOME");
    c.timeshare_path = std::string(home ? home : "/tmp") + "/timeshare";
  }
  c.jpeg_quality = std::max(10, std::min(100, c.jpeg_quality));
  c.queue_size = std::max(1, c.queue_size);
  if (c.trigger_period <= 0.0) c.trigger_period = 0.1;
  return ok;
}

}  // namespace huateng_camera_ros
