# huateng_camera_ros

ROS 1 **and** ROS 2 driver for **Huateng Vision** industrial cameras (MindVision "MVSDK", e.g. the GigE camera
**HT-GE134GC**). One package, built with catkin (ROS 1) or colcon (ROS 2), same configuration file for both.

- Free-running, software- or hardware-triggered
- Raw image (`sensor_msgs/Image`, color or mono8), JPEG (`sensor_msgs/CompressedImage`) and `CameraInfo`;
  only what is subscribed is converted
- Grab and publish run in separate threads, so a slow subscriber never blocks the camera
- Optional **LiDAR time sync**: with a hardware-synced LiDAR-camera rig (MCU trigger + PPS, as in
  [LIV_handhold](https://github.com/xuankuzcr/LIV_handhold)) the images are stamped in LiDAR time, on the trigger
  grid (no `img_time_offset` needed in e.g. FAST-LIVO2)
- `camera_info_manager` with `set_camera_info`, so `camera_calibration` can write the calibration directly

**Status:** The driver logic is in use on a Raspberry Pi 4 (ROS 1 Noetic in Docker) with an HT-GE134GC,
hardware-triggered at 10 Hz together with a Livox Avia. This package (ROS 1/ROS 2 in one) is build-tested on
Noetic, Humble and Jazzy (x86_64); a hardware test of this version is pending.

## 1. Install the SDK

The camera SDK is closed source and **not part of this repository**. It is a free download from the manufacturer
([huatengvision.com/download](https://www.huatengvision.com/download/1/), "HuaTeng Vision SDK for Linux", no login).
The script downloads it (V2.1.0.49, checksum verified) and installs headers and the library for your architecture
(x86_64, aarch64, armv7, i686) to `/opt/huatengvision_SDK`:

```bash
git clone https://github.com/Moritz1093/huateng_camera_ros.git
sudo huateng_camera_ros/scripts/install_sdk.sh
# or from an archive you downloaded yourself:
sudo huateng_camera_ros/scripts/install_sdk.sh linuxSDK_V2.1.0.49\(202511141513\).tar.gz
```

Another location: `sudo HUATENG_SDK_DIR=/my/dir scripts/install_sdk.sh` and build with the same `HUATENG_SDK_DIR`.

## 2. Build

The package detects the ROS version itself (`ROS_VERSION` from the ROS setup script).

**ROS 1 (Noetic)**
```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/Moritz1093/huateng_camera_ros.git
cd ~/catkin_ws
rosdep update --include-eol-distros     # Noetic is end-of-life, rosdep skips it otherwise
rosdep install --from-paths src --ignore-src -y
catkin build                            # or catkin_make
source devel/setup.bash
```

**ROS 2 (Humble, Jazzy)**
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Moritz1093/huateng_camera_ros.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## 3. Run

```bash
roslaunch huateng_camera_ros camera.launch                       # ROS 1
ros2 launch huateng_camera_ros camera.launch.py                  # ROS 2
# options: config:=lidar_sync   config_path:=/path/own.yaml   time_source:=host|lidar   log_frames:=40
```

| Topic | Type |
|---|---|
| `camera/image_raw` | `sensor_msgs/Image` (`rgb8`/`bgr8` or `mono8`) |
| `camera/image/compressed` | `sensor_msgs/CompressedImage` (JPEG, color) |
| `camera/camera_info` | `sensor_msgs/CameraInfo` |
| `camera/set_camera_info` | service of `camera_info_manager` |

Topic names, frame id etc. are set in the config file. Note that the JPEG topic is `camera/image/compressed`, not
the image_transport name `camera/image_raw/compressed`; for image_transport subscribers use the base topic
`camera/image` with transport `compressed`, or change `TopicNameCompressed`.

Every 10 s the driver logs a statistics line (`100 images received, 100 published, 0 triggers without image ...`).

## 4. Configuration

Two examples in [config/](config/), all keys are commented there:

| File | Use |
|---|---|
| [camera.yaml](config/camera.yaml) | free-running camera, host time stamps (default) |
| [lidar_sync.yaml](config/lidar_sync.yaml) | hardware trigger, time stamps in LiDAR time |

Most important keys: `TriggerMode` (`Continuous` / `Software` / `Hardware`), `TriggerSignal` (`RisingEdge` /
`FallingEdge`), `ExposureAuto`, `Orientation` (rotation in 90 deg steps), `RawEncoding` (`color` / `mono8`),
`JpegQuality`, `FrameSpeed` (GigE speed level, `-1` = highest), `SerialNumber` (several cameras),
`TimeSource` (`Host` / `LiDAR`), `CalibrationPath`.

## 5. LiDAR time sync (optional)

For LiDAR-camera fusion (e.g. [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)) the image stamps must be in
the LiDAR's clock. The hardware sync scheme is the one of [LIV_handhold](https://github.com/xuankuzcr/LIV_handhold)
(HKU MARS): an STM32 sends PPS + GPRMC to the LiDAR and triggers the camera on the same 100 ms grid.

**Needed**
- the MCU firmware and wiring from LIV_handhold (`stm32_timersync-open/`, PPS via RS485 for the Livox Avia)
- the `livox_ros_driver` from **[Moritz1093/LIV_handhold](https://github.com/Moritz1093/LIV_handhold)**: it writes
  the start of each LiDAR frame *and the host time* into the shared memory file `timeshare`. The original driver
  only writes the LiDAR time; this driver then has no host time and drops the images
- `config:=lidar_sync` (`TimeSource: LiDAR`, `TriggerMode: Hardware`), `TimeStampPath` = `timeshare_path` of the
  LiDAR driver (both default to `$HOME/timeshare`). In Docker, both drivers must see the same file

> **Limitation:** a LiDAR driver that writes the host time exists so far only for **ROS 1 and the Livox Avia**
> (`livox_ros_driver`, Livox SDK 1). `livox_ros_driver2` (MID-360, HAP, ROS 1/ROS 2) in LIV_handhold still writes
> only the LiDAR time. Under ROS 2, or with `livox_ros_driver2`, use `TimeSource: Host` until it gets the same change.

**How it works**
1. From `timeshare` the driver estimates the offset host clock ↔ LiDAR clock (minimum latency over a window).
2. The camera's own frame counter clock (`uiTimeStamp`) tells exactly how many trigger periods lie between two
   images, even if images are lost. This gives exactly periodic stamps.
3. The estimate of the exposure start is rounded to the trigger grid (`SnapToTriggerGrid`), because the MCU puts
   the trigger edge exactly on the PPS grid. The statistics line shows `estimate - grid ~18 ms` when healthy.

**Check the trigger edge once.** The camera's trigger input may be inverted (opto-coupler): then the camera's
`FallingEdge` is the MCU's rising edge. With the wrong edge the stamps are off by half a period (50 ms at 10 Hz).
Record LiDAR and camera, look at a fast rotation, or log raw values with `log_frames:=40`. Set `TriggerSignal`
explicitly, the camera keeps a changed setting until it loses power. `TriggerPhaseSec` shifts the grid if your
triggering edge is not on the PPS grid.

Without LiDAR time (LiDAR driver not running) images are dropped, unless `LidarFallbackHostTime: On`.

## 6. Calibration

```bash
# ROS 1
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.04 image:=/camera/image_raw camera:=/camera
# ROS 2
ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.04 --ros-args -r image:=/camera/image_raw -p camera:=/camera
```
**COMMIT** writes the result to `CalibrationPath` (or `~/.ros/camera_info/camera.yaml`). For calibration a
free-running camera with `TimeSource: Host` is enough.

## 7. Troubleshooting

| Problem | Check |
|---|---|
| `No camera found` | Camera and PC in the same subnet (set the camera IP with the SDK tool `GeConfigCmd` or the Windows tool), firewall, cable |
| Only ~6 fps | **Gigabit cable/switch** (a 100 Mbit link limits the rate), `FrameSpeed: -1`, MTU 9000 on the network card (`sudo ip link set dev eth0 mtu 9000`) |
| `triggers without image` | Exposure longer than the trigger period, or bandwidth (see above) |
| Images dropped, `without LiDAR time` | LiDAR driver running? Same `timeshare` path? Using the LiDAR driver from Moritz1093/LIV_handhold? |
| Stamps off by ~50 ms | wrong `TriggerSignal`, see section 5 |
| Camera already in use | another process (e.g. the Windows/Linux demo) holds the camera |

Tests without a camera: `g++ -std=c++17 -Iinclude test/test_trigger_clock.cpp -o t && ./t` (simulation of the time
stamp logic with lost images, latency, clock drift and an MCU restart).

## Origin and license

- Driver code: written from scratch on top of the Huateng/MindVision SDK. **MIT license**, see [LICENSE](LICENSE).
- LiDAR time sync: hardware scheme, MCU firmware and the `timeshare` format (`{int64 high; int64 low;}`) from
  [LIV_handhold](https://github.com/xuankuzcr/LIV_handhold) (GPL-3.0). Only the data layout is shared, no code is
  copied; the matching LiDAR driver is in the fork [Moritz1093/LIV_handhold](https://github.com/Moritz1093/LIV_handhold).
- Huateng Vision / MindVision SDK: closed source, property of the manufacturer, not included and not covered by the
  MIT license.
