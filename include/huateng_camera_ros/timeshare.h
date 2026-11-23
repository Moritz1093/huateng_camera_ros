// Shared memory "timeshare" between livox_ros_driver (writer) and this camera driver (reader).
//
// Same file as in https://github.com/Moritz1093/LIV_handhold (livox_ros_driver/livox_ros_driver/timeshare.h),
// format from https://github.com/xuankuzcr/LIV_handhold. Struct and functions must stay identical in both.
//
// A 16-byte file mapped with mmap. Layout unchanged from LIV_handhold: {int64 high; int64 low;}
//   low  = LiDAR/MCU time (ns) of the start of the most recently published LiDAR frame
//   high = host time (ns, ros::Time::now) when it was written; 0 = invalid or being written
//
// Write: high=0, low=..., high=... (all atomic). Read: high, low, high; valid only if both highs are equal and != 0
// (seqlock). This lets a camera driver estimate the host <-> LiDAR clock offset without a torn read.
// Readers that only use "low" (e.g. the original mvs_ros_driver) keep working unchanged.
#pragma once

#include <stdint.h>

struct time_stamp {
  int64_t high;
  int64_t low;
};

inline void timeshare_clear(time_stamp *p) {
  if (!p) return;
  __atomic_store_n(&p->high, (int64_t)0, __ATOMIC_SEQ_CST);
  __atomic_store_n(&p->low, (int64_t)0, __ATOMIC_SEQ_CST);
}

inline void timeshare_write(time_stamp *p, int64_t lidar_ns, int64_t host_ns) {
  if (!p) return;
  __atomic_store_n(&p->high, (int64_t)0, __ATOMIC_SEQ_CST);
  __atomic_store_n(&p->low, lidar_ns, __ATOMIC_SEQ_CST);
  __atomic_store_n(&p->high, host_ns, __ATOMIC_SEQ_CST);
}

inline bool timeshare_read(const time_stamp *p, int64_t *lidar_ns, int64_t *host_ns) {
  if (!p) return false;
  for (int i = 0; i < 3; ++i) {
    const int64_t h1 = __atomic_load_n(&p->high, __ATOMIC_SEQ_CST);
    const int64_t l = __atomic_load_n(&p->low, __ATOMIC_SEQ_CST);
    const int64_t h2 = __atomic_load_n(&p->high, __ATOMIC_SEQ_CST);
    if (h1 != 0 && h1 == h2) {
      *lidar_ns = l;
      *host_ns = h1;
      return true;
    }
  }
  return false;
}
