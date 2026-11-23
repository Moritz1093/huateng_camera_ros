// ROS 1 (catkin) wrapper around the driver core (driver.h).
#include <huateng_camera_ros/config.h>
#include <huateng_camera_ros/driver.h>

#include <boost/make_shared.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <string>

using namespace huateng_camera_ros;

namespace {

void rosLog(LogLevel l, const std::string &m) {
  switch (l) {
    case LogLevel::Debug: ROS_DEBUG("%s", m.c_str()); break;
    case LogLevel::Info: ROS_INFO("%s", m.c_str()); break;
    case LogLevel::Warn: ROS_WARN("%s", m.c_str()); break;
    case LogLevel::Error: ROS_ERROR("%s", m.c_str()); break;
    case LogLevel::Fatal: ROS_FATAL("%s", m.c_str()); break;
  }
}

class Ros1Node {
 public:
  Ros1Node(ros::NodeHandle &nh, const Config &cfg) : cfg_(cfg) {
    // CameraInfoManager offers <camera_name>/set_camera_info (camera_calibration with camera:=/<camera_name>)
    ros::NodeHandle cim_nh(nh, cfg_.camera_name);
    cim_.reset(new camera_info_manager::CameraInfoManager(cim_nh, cfg_.camera_name, cfg_.calibration_url));
    pub_raw_ = nh.advertise<sensor_msgs::Image>(cfg_.topic_raw, 2);
    pub_comp_ = nh.advertise<sensor_msgs::CompressedImage>(cfg_.topic_compressed, 2);
    pub_info_ = nh.advertise<sensor_msgs::CameraInfo>(cfg_.topic_camera_info, 2);
    ROS_INFO("Raw: %s (%s), JPEG: %s (quality %d)", pub_raw_.getTopic().c_str(), cfg_.raw_mono ? "mono8" : "color",
             pub_comp_.getTopic().c_str(), cfg_.jpeg_quality);

    Driver::Callbacks cb;
    cb.log = rosLog;
    cb.want_image = [this] { return pub_raw_.getNumSubscribers() > 0 || pub_comp_.getNumSubscribers() > 0; };
    cb.publish = [this](const Frame &f) { publish(f); };
    driver_.reset(new Driver(cfg_, cb));
    stats_timer_ = nh.createWallTimer(ros::WallDuration(10.0), [this](const ros::WallTimerEvent &) {
      ROS_INFO("10 s: %s", driver_->statsLine().c_str());
    });
  }

  bool start() { return driver_->start(); }
  void stop() { driver_->stop(); }

 private:
  void publish(const Frame &f) {
    ros::Time stamp;
    stamp.fromNSec(static_cast<uint64_t>(f.stamp_ns));
    if (!f.image.empty()) {
      const int w = f.image.cols, h = f.image.rows;
      if (pub_raw_.getNumSubscribers() > 0) {
        auto msg = boost::make_shared<sensor_msgs::Image>();
        msg->header.stamp = stamp;
        msg->header.frame_id = cfg_.frame_id;
        msg->height = h;
        msg->width = w;
        msg->is_bigendian = 0;
        if (cfg_.raw_mono) {
          msg->encoding = sensor_msgs::image_encodings::MONO8;
          msg->step = w;
          msg->data.resize(static_cast<size_t>(w) * h);
          cv::Mat gray(h, w, CV_8UC1, msg->data.data());
          toMono(f, gray);
        } else {
          msg->encoding = f.is_rgb ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::BGR8;
          msg->step = w * 3;
          const uint8_t *d = colorData(f);
          msg->data.assign(d, d + static_cast<size_t>(w) * h * 3);
        }
        pub_raw_.publish(msg);
      }
      if (pub_comp_.getNumSubscribers() > 0) {
        auto msg = boost::make_shared<sensor_msgs::CompressedImage>();
        msg->header.stamp = stamp;
        msg->header.frame_id = cfg_.frame_id;
        msg->format = "jpeg";
        if (toJpeg(f, cfg_.jpeg_quality, msg->data)) pub_comp_.publish(msg);
      }
      last_w_ = w;
      last_h_ = h;
    }
    auto ci = boost::make_shared<sensor_msgs::CameraInfo>(cim_->getCameraInfo());
    ci->header.stamp = stamp;
    ci->header.frame_id = cfg_.frame_id;
    if (last_w_ > 0) {
      ci->width = last_w_;
      ci->height = last_h_;
    }
    pub_info_.publish(ci);
  }

  Config cfg_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cim_;
  ros::Publisher pub_raw_, pub_comp_, pub_info_;
  ros::WallTimer stats_timer_;
  std::unique_ptr<Driver> driver_;
  int last_w_ = 0, last_h_ = 0;  // only used in the publish thread
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string config_path;
  pnh.param<std::string>("config_path", config_path, "");
  Config cfg;
  if (config_path.empty()) {
    ROS_WARN("Parameter ~config_path not set, using built-in defaults");
  } else {
    loadConfig(config_path, cfg, rosLog);
  }
  std::string ts;
  if (pnh.getParam("time_source", ts) && !ts.empty() && !parseTimeSource(ts, cfg.time_source)) {
    ROS_WARN("Unknown ~time_source '%s' (host / lidar), keeping %s", ts.c_str(), toString(cfg.time_source));
  }
  pnh.param("log_frames", cfg.log_frames, cfg.log_frames);

  Ros1Node node(nh, cfg);
  if (!node.start()) return 1;
  ros::spin();
  node.stop();
  return 0;
}
