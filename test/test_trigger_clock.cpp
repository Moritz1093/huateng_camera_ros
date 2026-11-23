// Simulation test for trigger_clock.h, without ROS and without camera.
//
//   g++ -std=c++17 -O2 -Iinclude test/test_trigger_clock.cpp -o test_trigger_clock && ./test_trigger_clock [-v]
//
// Simulates: 10 Hz trigger with 50 ms phase to the LiDAR grid, camera clock with offset and 80 ppm drift,
// host clock with offset and 50 ppm drift to the MCU, LiDAR publishing 100 ms + 5..250 ms after frame start,
// image latency exposure + transfer + queue 0..400 ms, 40 % lost images, MCU restart after 120 s.
// Checks: error (stamp - true trigger time) is stable except for a constant residual, no 100 ms jumps.
#include <huateng_camera_ros/trigger_clock.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using huateng_camera_ros::HostMcuOffset;
using huateng_camera_ros::TriggerClock;

int main(int argc, char **argv) {
  const bool verbose = argc > 1 && std::string(argv[1]) == "-v";
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> u(0.0, 1.0);

  const int64_t P = 100000000LL;            // 100 ms
  const int64_t phase = 50000000LL;         // trigger 50 ms after the LiDAR grid
  const int64_t exp_ns = 8000000LL;         // 8 ms exposure
  const int64_t transfer_ns = 12000000LL;   // 12 ms GigE

  HostMcuOffset offset;
  TriggerClock clock;

  int64_t mcu0 = 1590199200LL * 1000000000LL;  // fake MCU time (2020-05-23)
  const double host_minus_mcu_s = 201442230.5;
  const double host_drift = 50e-6, cam_drift = 80e-6;
  const int64_t restart_at = 1200;             // trigger index of the MCU restart (120 s)

  std::vector<double> err_ms;
  int64_t resets = 0, missed = 0, frames = 0;
  int64_t next_lidar = 0;

  const int64_t host0 = static_cast<int64_t>((host_minus_mcu_s) * 1e9) + mcu0;  // host time at real time 0

  for (int64_t i = 0; i < 3000; ++i) {
    // MCU restart: MCU time jumps back, host and camera clock keep running
    const bool second = i >= restart_at;
    const int64_t mcu_base = second ? mcu0 - 3600LL * 1000000000LL : mcu0;
    const int64_t epoch_real = second ? restart_at * P : 0;  // real time (ns) of the epoch start
    const int64_t idx = second ? i - restart_at : i;
    const int64_t t_mcu = mcu_base + phase + idx * P;  // true trigger time (= exposure start)
    const double t_abs_s = i * 0.1;                    // continuous real time for the camera clock
    // MCU time -> real time -> host time (with drift)
    auto host_of = [&](int64_t mcu_ns) {
      const double real = static_cast<double>(epoch_real + (mcu_ns - mcu_base));
      return host0 + static_cast<int64_t>(real * (1.0 + host_drift));
    };

    // publish LiDAR frames up to this point (frame start on the grid)
    while (next_lidar <= idx) {
      const int64_t f_start = mcu_base + next_lidar * P;
      const int64_t pub_lat = 5000000LL + static_cast<int64_t>(u(rng) * 245000000.0);
      offset.add(f_start, host_of(f_start + P + pub_lat));
      ++next_lidar;
    }
    if (i == restart_at - 1) next_lidar = 0;

    if (u(rng) < 0.4) continue;  // image lost

    const uint32_t cam_ts =
        static_cast<uint32_t>(static_cast<int64_t>((1234.5 + t_abs_s * (1.0 + cam_drift)) * 1e4));
    const double q = u(rng);
    const int64_t queue_ns = q < 0.2 ? 0 : static_cast<int64_t>(u(rng) * 400000000.0);
    const int64_t recv_host = host_of(t_mcu + exp_ns + transfer_ns + queue_ns);

    const auto r = clock.update(cam_ts, recv_host - exp_ns - transfer_ns);
    const int64_t stamp = offset.toMcu(r.stamp_ns);
    ++frames;
    missed += r.missed;
    if (r.reset) ++resets;
    // do not rate settling after start/MCU restart (offset window fills, ~2 s)
    if (idx > 50) err_ms.push_back((stamp - t_mcu) / 1e6);
    if (verbose && i % 100 == 0) {
      const int64_t true_off = host_of(t_mcu) - t_mcu;
      std::printf("i=%5lld  error %7.2f ms  offset error %7.2f ms\n", (long long)i, (stamp - t_mcu) / 1e6,
                  (offset.offset() - true_off) / 1e6);
    }
  }

  double mn = 1e9, mx = -1e9, sum = 0;
  for (double e : err_ms) { mn = std::min(mn, e); mx = std::max(mx, e); sum += e; }
  const double mean = sum / err_ms.size();
  std::printf("Images: %lld, skipped triggers detected: %lld, chain restarts: %lld\n",
              (long long)frames, (long long)missed, (long long)resets);
  std::printf("Error stamp - trigger (ms): min %.2f  mean %.2f  max %.2f  span %.2f\n", mn, mean, mx, mx - mn);
  std::printf("Host-MCU jumps detected: %lld\n", (long long)offset.jumps());

  // Expected: exactly one chain start (the MCU restart is handled via the offset) and one offset jump
  bool ok = (mx - mn) < 10.0 && resets == 1 && offset.jumps() == 1;
  std::printf("%s\n", ok ? "OK" : "FAILED");
  return ok ? 0 : 1;
}
