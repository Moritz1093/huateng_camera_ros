// Time stamps for hardware-triggered camera images.
//
// Problem: the time between trigger and "image received" varies a lot (exposure, GigE transfer, SDK queue, CPU load).
// Stamping with "now" or with "the latest LiDAR frame" is therefore off by changing amounts.
//
// Solution:
//  1. HostMcuOffset: offset host clock minus MCU/LiDAR clock from timeshare (minimum over a window = lowest latency).
//  2. TriggerClock: the difference of two images in the camera's own clock (uiTimeStamp, 0.1 ms) tells exactly how
//     many trigger periods lay in between, even if images were lost. Image i is at base + k_i * period. base is the
//     minimum of (estimated host time - k_i * period) over a window, i.e. the value of the least delayed image.
//     Result: exactly periodic stamps with a constant residual offset.
//     The chain runs in host time; only the result is converted to MCU time with HostMcuOffset. Improvements of the
//     offset (settling, MCU restart) therefore take effect immediately instead of being stuck in the window.
//
// No ROS dependency, so it can be tested without ROS (test/test_trigger_clock.cpp).
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>

namespace huateng_camera_ros {

// Round to the grid phase + n * period
inline int64_t snapToGrid(int64_t t, int64_t period, int64_t phase) {
  const int64_t a = t - phase + period / 2;
  int64_t q = a / period;
  if ((a % period != 0) && (a < 0)) --q;
  return phase + q * period;
}

// Offset host clock minus MCU clock.
// The Livox driver writes (frame_start, host_write) when a frame is published:
//   host_write = frame_start + frame_duration + offset + latency,  latency >= 0
// Estimate: minimum over a window (= frame with the lowest latency), then a low-pass filter, because the minimum
// would otherwise jump by a few ms depending on which frame is in the window.
class HostMcuOffset {
 public:
  explicit HostMcuOffset(int64_t frame_ns = 100000000LL, std::size_t window = 200,
                         double alpha = 0.02, int64_t jump_ns = 1000000000LL)
      : frame_ns_(frame_ns), window_(window), alpha_(alpha), jump_ns_(jump_ns) {}

  // Returns true if a new frame was taken over (the same frame_start is ignored).
  bool add(int64_t frame_start_ns, int64_t host_write_ns) {
    if (have_last_ && frame_start_ns == last_frame_ns_) return false;
    have_last_ = true;
    last_frame_ns_ = frame_start_ns;
    last_host_write_ns_ = host_write_ns;
    const int64_t s = host_write_ns - frame_start_ns - frame_ns_;
    // Jump of the MCU time (MCU restarted) or of the host time (NTP): drop the old window
    const bool jump = !samples_.empty() && std::llabs(s - offset_ns_) > jump_ns_;
    if (jump) {
      samples_.clear();
      ++jumps_;
    }
    const bool first = samples_.empty();
    samples_.push_back(s);
    if (samples_.size() > window_) samples_.pop_front();
    const int64_t raw = *std::min_element(samples_.begin(), samples_.end());
    if (first) {
      filtered_ = static_cast<double>(raw);
    } else if (raw < offset_ns_ && samples_.size() < window_) {
      filtered_ = static_cast<double>(raw);  // settling: take better values immediately
    } else {
      filtered_ += alpha_ * (static_cast<double>(raw) - filtered_);
    }
    offset_ns_ = static_cast<int64_t>(filtered_);
    return true;
  }

  bool valid() const { return !samples_.empty(); }
  std::size_t samples() const { return samples_.size(); }
  int64_t offset() const { return offset_ns_; }
  int64_t lastHostWrite() const { return last_host_write_ns_; }
  int64_t toMcu(int64_t host_ns) const { return host_ns - offset_ns_; }
  int64_t jumps() const { return jumps_; }

 private:
  int64_t frame_ns_;
  std::size_t window_;
  double alpha_;
  int64_t jump_ns_;
  std::deque<int64_t> samples_;
  double filtered_ = 0.0;
  int64_t offset_ns_ = 0;
  bool have_last_ = false;
  int64_t last_frame_ns_ = 0;
  int64_t last_host_write_ns_ = 0;
  int64_t jumps_ = 0;
};

class TriggerClock {
 public:
  struct Params {
    int64_t period_ns = 100000000LL;        // trigger period (10 Hz)
    std::size_t window = 300;               // images for the minimum (300 = 30 s)
    int64_t reset_gap_ns = 30000000000LL;   // longer pause in the camera clock: start over
    int64_t jump_ns = 1000000000LL;         // estimate jumps by more than this: start over
  };

  struct Result {
    int64_t stamp_ns = 0;
    int64_t missed = 0;    // trigger pulses skipped since the last image
    bool reset = false;    // chain restarted
  };

  TriggerClock() = default;
  explicit TriggerClock(const Params &p) : p_(p) {}

  // cam_ts_100us: uiTimeStamp from tSdkFrameHead (camera clock, 0.1 ms, wraps after ~5 days)
  // est_host_ns:  estimate of the exposure start in host time (received - exposure - transfer).
  //               May be late (queue), but never early.
  // The result stamp_ns is in host time; convert to MCU time with HostMcuOffset::toMcu.
  Result update(uint32_t cam_ts_100us, int64_t est_host_ns) {
    Result r;
    if (!started_) {
      restart(cam_ts_100us);
      r.reset = true;
    } else {
      const uint32_t d = cam_ts_100us - last_cam_;  // modulo 2^32, so wrap-around does not matter
      const int64_t d_ns = static_cast<int64_t>(d) * 100000LL;
      if (d_ns > p_.reset_gap_ns) {
        restart(cam_ts_100us);
        r.reset = true;
      } else {
        int64_t n = (d_ns + p_.period_ns / 2) / p_.period_ns;
        if (n < 1) n = 1;  // duplicate camera time: count as next period
        k_ += n;
        r.missed = n - 1;
        last_cam_ = cam_ts_100us;
      }
    }

    int64_t v = est_host_ns - k_ * p_.period_ns;
    if (!win_.empty() && std::llabs(v - base_) > p_.jump_ns) {
      // host time jumped (e.g. NTP) or camera clock implausible: restart the chain
      restart(cam_ts_100us);
      r.reset = true;
      v = est_host_ns;
    }
    win_.push_back(v);
    if (win_.size() > p_.window) win_.pop_front();
    base_ = *std::min_element(win_.begin(), win_.end());

    r.stamp_ns = base_ + k_ * p_.period_ns;
    return r;
  }

  int64_t base() const { return base_; }

 private:
  void restart(uint32_t cam_ts_100us) {
    started_ = true;
    last_cam_ = cam_ts_100us;
    k_ = 0;
    win_.clear();
  }

  Params p_;
  bool started_ = false;
  uint32_t last_cam_ = 0;
  int64_t k_ = 0;
  std::deque<int64_t> win_;
  int64_t base_ = 0;
};

}  // namespace huateng_camera_ros
