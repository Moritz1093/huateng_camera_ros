#include <huateng_camera_ros/driver.h>
#include <huateng_camera_ros/timeshare.h>
#include <huateng_camera_ros/trigger_clock.h>

#include "CameraApi.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <chrono>
#include <cstdio>
#include <map>

namespace huateng_camera_ros {
namespace {

constexpr int64_t kNsPerSec = 1000000000LL;

int64_t hostNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

double msSince(const std::chrono::steady_clock::time_point &t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

std::string fmt(const char *f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

// Log at most once per period per key
class Throttle {
 public:
  bool operator()(const char *key, double period_s) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(m_);
    auto it = last_.find(key);
    if (it != last_.end() && std::chrono::duration<double>(now - it->second).count() < period_s) return false;
    last_[key] = now;
    return true;
  }

 private:
  std::mutex m_;
  std::map<std::string, std::chrono::steady_clock::time_point> last_;
};

// Read-only access to timeshare; retried while the LiDAR driver has not created the file yet
class TimeShareReader {
 public:
  TimeShareReader(std::string path, const LogFn &log, Throttle &throttle)
      : path_(std::move(path)), log_(log), throttle_(throttle) {}
  ~TimeShareReader() {
    if (p_) munmap(const_cast<time_stamp *>(p_), sizeof(time_stamp));
  }

  bool read(int64_t *lidar_ns, int64_t *host_ns) {
    if (!p_ && !tryOpen()) return false;
    return timeshare_read(p_, lidar_ns, host_ns);
  }

 private:
  bool tryOpen() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_try_ < std::chrono::seconds(1)) return false;
    last_try_ = now;
    const int fd = open(path_.c_str(), O_RDONLY);
    if (fd < 0) {
      if (throttle_("ts_missing", 10.0))
        log_(LogLevel::Warn, "timeshare " + path_ + " does not exist yet (LiDAR driver running?)");
      return false;
    }
    struct stat st {};
    if (fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(time_stamp))) {
      close(fd);
      if (throttle_("ts_small", 10.0)) log_(LogLevel::Warn, "timeshare " + path_ + " is too small");
      return false;
    }
    void *m = mmap(nullptr, sizeof(time_stamp), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) {
      if (throttle_("ts_mmap", 10.0)) log_(LogLevel::Error, "mmap of " + path_ + " failed");
      return false;
    }
    p_ = static_cast<const time_stamp *>(m);
    log_(LogLevel::Info, "timeshare mapped: " + path_);
    return true;
  }

  std::string path_;
  const LogFn &log_;
  Throttle &throttle_;
  const time_stamp *p_ = nullptr;
  std::chrono::steady_clock::time_point last_try_{};
};

struct Stats {
  std::atomic<uint64_t> received{0}, published{0}, missed_trigger{0}, dropped_queue{0}, unstamped{0};
  std::atomic<uint64_t> isp_us{0}, isp_n{0}, pub_us{0}, pub_n{0};
};

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
void toMono(const Frame &f, cv::Mat &dst) {
  cv::cvtColor(f.image, dst, f.is_rgb ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
}

const uint8_t *colorData(const Frame &f) { return f.image.data; }

bool toJpeg(const Frame &f, int quality, std::vector<uint8_t> &out) {
  cv::Mat bgr;
  if (f.is_rgb) {
    cv::cvtColor(f.image, bgr, cv::COLOR_RGB2BGR);
  } else {
    bgr = f.image;
  }
  return cv::imencode(".jpg", bgr, out, {cv::IMWRITE_JPEG_QUALITY, quality});
}

// ---------------------------------------------------------------------------------------------------------------------
struct Driver::Impl {
  Impl(const Config &c, Callbacks callbacks)
      : cfg(c), cb(std::move(callbacks)), timeshare(c.timeshare_path, cb.log, throttle),
        offset(static_cast<int64_t>(c.trigger_period * kNsPerSec)) {
    log_frames_left = cfg.log_frames;
    TriggerClock::Params p;
    p.period_ns = static_cast<int64_t>(cfg.trigger_period * kNsPerSec);
    clock = TriggerClock(p);
  }

  void log(LogLevel l, const std::string &m) { cb.log(l, m); }

  // -------------------------------------------------------------------------------------------------------------------
  bool openCamera() {
    if (CameraSdkInit(0) != CAMERA_STATUS_SUCCESS) {  // 0 = English SDK messages, 1 = Chinese
      log(LogLevel::Fatal, "Camera SDK could not be initialized");
      return false;
    }
    tSdkCameraDevInfo list[16];
    int count = 16;
    if (CameraEnumerateDevice(list, &count) != CAMERA_STATUS_SUCCESS || count < 1) {
      log(LogLevel::Fatal, "No camera found (check network/GigE, see README \"Troubleshooting\")");
      return false;
    }
    int idx = 0;
    if (!cfg.serial.empty()) {
      idx = -1;
      for (int i = 0; i < count; ++i) {
        if (cfg.serial == list[i].acSn) idx = i;
      }
      if (idx < 0) {
        std::string found;
        for (int i = 0; i < count; ++i) found += std::string(i ? ", " : "") + list[i].acSn;
        log(LogLevel::Fatal, "Camera with serial " + cfg.serial + " not found (found: " + found + ")");
        return false;
      }
    }
    const int st = CameraInit(&list[idx], -1, -1, &cam);
    if (st != CAMERA_STATUS_SUCCESS) {
      log(LogLevel::Fatal, fmt("Camera could not be opened: error %d (already in use?)", st));
      return false;
    }
    sdk_open = true;
    log(LogLevel::Info, fmt("Camera: %s (SN %s), %d camera(s) found", list[idx].acFriendlyName, list[idx].acSn, count));

    CameraSetAeState(cam, cfg.exposure_auto ? TRUE : FALSE);
    if (cfg.exposure_auto) {
      CameraSetAeTarget(cam, cfg.exposure_target);
    } else {
      CameraSetExposureTime(cam, cfg.exposure_time_us);
    }
    CameraSetTriggerMode(cam, cfg.trigger_mode);
    if (!cfg.trigger_signal.empty()) {
      const std::string s = lower(cfg.trigger_signal);
      CameraSetExtTrigSignalType(cam, s == "fallingedge" ? EXT_TRIG_TRAILING_EDGE : EXT_TRIG_LEADING_EDGE);
    }
    CameraSetWbMode(cam, cfg.wb_auto ? TRUE : FALSE);
    CameraSetRotate(cam, cfg.orientation);
    CameraSetIspOutFormat(cam, cfg.isp_bgr ? CAMERA_MEDIA_TYPE_BGR8 : CAMERA_MEDIA_TYPE_RGB8);
    if (cfg.gamma >= 0) CameraSetGamma(cam, cfg.gamma);
    is_rgb = !cfg.isp_bgr;

    static const char *kTrig[] = {"continuous", "software", "hardware"};
    log(LogLevel::Info,
        fmt("AE %s (target %d), trigger %s%s%s, WB %s, rotation %d, gamma %d", cfg.exposure_auto ? "on" : "off",
            cfg.exposure_target, kTrig[std::max(0, std::min(2, cfg.trigger_mode))],
            cfg.trigger_signal.empty() ? "" : " / ", cfg.trigger_signal.c_str(), cfg.wb_auto ? "auto" : "manual",
            cfg.orientation, cfg.gamma));
    if (cfg.trigger_mode == 2) {
      INT sig = -1;
      UINT trig_delay = 0, ext_delay = 0, jitter = 0;
      CameraGetExtTrigSignalType(cam, &sig);
      CameraGetTriggerDelayTime(cam, &trig_delay);
      CameraGetExtTrigDelayTime(cam, &ext_delay);
      CameraGetExtTrigJitterTime(cam, &jitter);
      static const char *kSig[] = {"rising edge", "falling edge", "high level", "low level"};
      log(LogLevel::Info,
          fmt("Trigger input: %s, delay %u us (external %u us), debounce %u us | grid %s, phase %.1f ms",
              (sig >= 0 && sig <= 3) ? kSig[sig] : "?", trig_delay, ext_delay, jitter,
              (cfg.time_source == TimeSource::Lidar && cfg.snap_to_grid) ? "on" : "off", cfg.trigger_phase * 1e3));
    }

    tSdkCameraCapbility cap;
    if (CameraGetCapability(cam, &cap) != CAMERA_STATUS_SUCCESS) {
      log(LogLevel::Fatal, "CameraGetCapability failed");
      return false;
    }
    max_bytes = static_cast<size_t>(cap.sResolutionRange.iWidthMax) * cap.sResolutionRange.iHeightMax * 3;

    // Speed level (low/normal/high/super). Sets the transfer rate over GigE. In "normal" the HT-GE134GC only
    // reaches ~6 fps, therefore default: highest level
    int speed_before = -1;
    CameraGetFrameSpeed(cam, &speed_before);
    std::string speeds;
    for (int i = 0; i < cap.iFrameSpeedDesc; ++i) {
      speeds += (i ? ", " : "") + std::to_string(cap.pFrameSpeedDesc[i].iIndex) + "=" + cap.pFrameSpeedDesc[i].acDescription;
    }
    if (cap.iFrameSpeedDesc > 0) {
      const int want = cfg.frame_speed < 0 ? cap.iFrameSpeedDesc - 1 : std::min(cfg.frame_speed, cap.iFrameSpeedDesc - 1);
      CameraSetFrameSpeed(cam, want);
    }
    int speed_now = -1, pack = -1;
    CameraGetFrameSpeed(cam, &speed_now);
    CameraGetTransPackLen(cam, &pack);
    log(LogLevel::Info, fmt("Speed: %d -> %d (levels: %s), packet length index %d (%u bytes)", speed_before, speed_now,
                            speeds.c_str(), pack,
                            (pack >= 0 && pack < cap.iPackLenDesc) ? cap.pPackLenDesc[pack].iPackSize : 0u));
    return true;
  }

  // -------------------------------------------------------------------------------------------------------------------
  void grabLoop() {
    const int64_t transfer_ns = static_cast<int64_t>(cfg.transfer_latency * kNsPerSec);
    const bool lidar_mode = cfg.time_source == TimeSource::Lidar;
    while (running) {
      tSdkFrameHead head;
      BYTE *raw = nullptr;
      const int st = CameraGetImageBuffer(cam, &head, &raw, 1000);
      if (st != CAMERA_STATUS_SUCCESS) {
        if (st == CAMERA_STATUS_TIME_OUT) {
          if (throttle("timeout", 10.0)) log(LogLevel::Warn, "No image for 1 s (trigger active?)");
        } else if (throttle("grab_err", 5.0)) {
          log(LogLevel::Warn, fmt("CameraGetImageBuffer: error %d", st));
        }
        continue;
      }
      const int64_t recv_ns = hostNowNs();
      ++stats.received;
      if (log_frames_left > 0) {
        --log_frames_left;
        log(LogLevel::Info, fmt("Image: cam_ts %u (d %d), exp %u us, trig %d, recv %.4f (d %.1f ms)", head.uiTimeStamp,
                                static_cast<int>(head.uiTimeStamp - last_log_cam), head.uiExpTime,
                                static_cast<int>(head.bIsTrigger), recv_ns * 1e-9, (recv_ns - last_log_recv) * 1e-6));
        last_log_cam = head.uiTimeStamp;
        last_log_recv = recv_ns;
      }

      // 1) LiDAR mode: follow the host <-> MCU offset from timeshare
      bool lidar_ok = false;
      if (lidar_mode) {
        int64_t lidar_ns = 0, write_ns = 0;
        if (timeshare.read(&lidar_ns, &write_ns) && offset.add(lidar_ns, write_ns)) {
          offset_for_log = offset.offset();
        }
        lidar_ok = offset.valid() && (recv_ns - offset.lastHostWrite()) < 2 * kNsPerSec;
      }

      // 2) Stamp = exposure start (host time); with hardware trigger via the trigger chain
      const int64_t exp_ns = static_cast<int64_t>(head.uiExpTime) * 1000;
      int64_t stamp_host = recv_ns - exp_ns - transfer_ns;
      if (cfg.trigger_mode == 2) {
        const auto r = clock.update(head.uiTimeStamp, stamp_host);
        stats.missed_trigger += static_cast<uint64_t>(r.missed);
        if (r.reset && stats.received > 1) log(LogLevel::Info, "Trigger chain restarted");
        stamp_host = r.stamp_ns;
      }

      Frame f;
      f.is_rgb = is_rgb;
      if (lidar_ok) {
        int64_t stamp_mcu = offset.toMcu(stamp_host);
        if (cfg.trigger_mode == 2 && cfg.snap_to_grid) {
          // The MCU puts the trigger edge exactly on the PPS/LiDAR grid (+ phase). The estimate above is only
          // accurate to a few ms (unknown transfer/readout time) -> round to the nearest grid point
          const int64_t p = static_cast<int64_t>(cfg.trigger_period * kNsPerSec);
          const int64_t ph = static_cast<int64_t>(cfg.trigger_phase * kNsPerSec);
          const int64_t snapped = snapToGrid(stamp_mcu, p, ph);
          grid_dev_sum_ns += stamp_mcu - snapped;
          ++grid_dev_n;
          stamp_mcu = snapped;
        }
        if (cfg.stamp_mid_exposure) stamp_mcu += exp_ns / 2;
        f.stamp_ns = stamp_mcu;
      } else if (!lidar_mode || cfg.lidar_fallback_host) {
        if (lidar_mode && throttle("fallback", 30.0))
          log(LogLevel::Warn, "No LiDAR time: stamping with host time (LidarFallbackHostTime)");
        if (cfg.stamp_mid_exposure) stamp_host += exp_ns / 2;
        f.stamp_ns = stamp_host;
      } else {
        ++stats.unstamped;
        if (throttle("unstamped", 5.0))
          log(LogLevel::Warn, "No current LiDAR time in timeshare, image dropped (LiDAR driver running?)");
        CameraReleaseImageBuffer(cam, raw);
        continue;
      }

      // 3) ISP only if somebody wants an image
      if (cb.want_image()) {
        const auto t0 = std::chrono::steady_clock::now();
        f.buffer.create(1, static_cast<int>(max_bytes), CV_8UC1);
        const int pst = CameraImageProcess(cam, raw, f.buffer.data, &head);
        CameraReleaseImageBuffer(cam, raw);
        if (pst != CAMERA_STATUS_SUCCESS) {
          if (throttle("isp", 5.0)) log(LogLevel::Warn, fmt("CameraImageProcess: error %d", pst));
          continue;
        }
        f.image = cv::Mat(head.iHeight, head.iWidth, CV_8UC3, f.buffer.data);
        stats.isp_us += static_cast<uint64_t>(msSince(t0) * 1000.0);
        ++stats.isp_n;
      } else {
        CameraReleaseImageBuffer(cam, raw);
      }

      // 4) Hand over to the publish thread; if it is too slow, the oldest frame is dropped
      {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (queue.size() >= static_cast<size_t>(cfg.queue_size)) {
          queue.pop_front();
          ++stats.dropped_queue;
        }
        queue.push_back(std::move(f));
      }
      queue_cv.notify_one();
    }
  }

  void publishLoop() {
    while (true) {
      Frame f;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [this] { return !queue.empty() || !running; });
        if (queue.empty()) return;  // only when !running
        f = std::move(queue.front());
        queue.pop_front();
      }
      const auto t0 = std::chrono::steady_clock::now();
      cb.publish(f);
      stats.pub_us += static_cast<uint64_t>(msSince(t0) * 1000.0);
      ++stats.pub_n;
      ++stats.published;
    }
  }

  Config cfg;
  Callbacks cb;
  Throttle throttle;
  CameraHandle cam = 0;
  bool sdk_open = false;
  size_t max_bytes = 0;
  bool is_rgb = true;

  // used only in the grab thread
  TimeShareReader timeshare;
  HostMcuOffset offset;
  TriggerClock clock;
  int log_frames_left = 0;
  uint32_t last_log_cam = 0;
  int64_t last_log_recv = 0;

  std::atomic<bool> running{false};
  std::thread grab_thread, pub_thread;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<Frame> queue;
  Stats stats;
  std::atomic<int64_t> offset_for_log{0};
  std::atomic<int64_t> grid_dev_sum_ns{0}, grid_dev_n{0};
};

// ---------------------------------------------------------------------------------------------------------------------
Driver::Driver(const Config &cfg, Callbacks cb) : impl_(new Impl(cfg, std::move(cb))) {}

Driver::~Driver() { stop(); }

bool Driver::start() {
  Impl &d = *impl_;
  if (!d.openCamera()) {
    if (d.sdk_open) CameraUnInit(d.cam);
    d.sdk_open = false;
    return false;
  }
  d.log(LogLevel::Info, std::string("Time source: ") + toString(d.cfg.time_source) +
                            (d.cfg.time_source == TimeSource::Lidar ? " (timeshare " + d.cfg.timeshare_path + ")" : ""));
  if (CameraPlay(d.cam) != CAMERA_STATUS_SUCCESS) {
    d.log(LogLevel::Fatal, "CameraPlay failed");
    CameraUnInit(d.cam);
    d.sdk_open = false;
    return false;
  }
  d.running = true;
  d.pub_thread = std::thread(&Impl::publishLoop, &d);
  d.grab_thread = std::thread(&Impl::grabLoop, &d);
  return true;
}

void Driver::stop() {
  Impl &d = *impl_;
  if (d.running.exchange(false)) {
    d.queue_cv.notify_all();
    if (d.grab_thread.joinable()) d.grab_thread.join();
    if (d.pub_thread.joinable()) d.pub_thread.join();
  }
  if (d.sdk_open) {
    CameraUnInit(d.cam);
    d.sdk_open = false;
    d.log(LogLevel::Info, "Camera closed");
  }
}

std::string Driver::statsLine() {
  Impl &d = *impl_;
  Stats &s = d.stats;
  const uint64_t rx = s.received.exchange(0), pub = s.published.exchange(0);
  const uint64_t miss = s.missed_trigger.exchange(0), drop = s.dropped_queue.exchange(0);
  const uint64_t unst = s.unstamped.exchange(0);
  const uint64_t isp_n = s.isp_n.exchange(0), isp_us = s.isp_us.exchange(0);
  const uint64_t pub_n = s.pub_n.exchange(0), pub_us = s.pub_us.exchange(0);
  const int64_t dev_n = d.grid_dev_n.exchange(0), dev_sum = d.grid_dev_sum_ns.exchange(0);
  std::string line = fmt("%lu images received, %lu published, %lu triggers without image, %lu dropped in queue | "
                         "ISP %.1f ms, output %.1f ms",
                         static_cast<unsigned long>(rx), static_cast<unsigned long>(pub),
                         static_cast<unsigned long>(miss), static_cast<unsigned long>(drop),
                         isp_n ? isp_us / 1000.0 / isp_n : 0.0, pub_n ? pub_us / 1000.0 / pub_n : 0.0);
  if (d.cfg.time_source == TimeSource::Lidar) {
    line += fmt(" | %lu without LiDAR time, host-MCU %.3f s, estimate - grid %.1f ms", static_cast<unsigned long>(unst),
                d.offset_for_log.load() / 1e9, dev_n ? dev_sum / 1e6 / dev_n : 0.0);
  }
  return line;
}

}  // namespace huateng_camera_ros
