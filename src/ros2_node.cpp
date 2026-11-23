// ROS 2 (ament) wrapper around the driver core (driver.h).
#include <huateng_camera_ros/config.h>
#include <huateng_camera_ros/driver.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

using namespace huateng_camera_ros;

namespace {

class Ros2Node : public rclcpp::Node {
 public:
  Ros2Node() : rclcpp::Node("camera_driver") {
    const std::string config_path = declare_parameter<std::string>("config_path", "");
    const std::string ts = declare_parameter<std::string>("time_source", "");
    const int log_frames = declare_parameter<int>("log_frames", 0);

    LogFn log = [this](LogLevel l, const std::string &m) {
      switch (l) {
        case LogLevel::Debug: RCLCPP_DEBUG(get_logger(), "%s", m.c_str()); break;
        case LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", m.c_str()); break;
        case LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", m.c_str()); break;
        case LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", m.c_str()); break;
        case LogLevel::Fatal: RCLCPP_FATAL(get_logger(), "%s", m.c_str()); break;
      }
    };
    if (config_path.empty()) {
      RCLCPP_WARN(get_logger(), "Parameter config_path not set, using built-in defaults");
    } else {
      loadConfig(config_path, cfg_, log);
    }
    if (!ts.empty() && !parseTimeSource(ts, cfg_.time_source)) {
      RCLCPP_WARN(get_logger(), "Unknown time_source '%s' (host / lidar), keeping %s", ts.c_str(),
                  toString(cfg_.time_source));
    }
    if (log_frames > 0) cfg_.log_frames = log_frames;

    // Same QoS as the ROS 1 version (reliable), so rosbag records every image
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(2));
    pub_raw_ = create_publisher<sensor_msgs::msg::Image>(cfg_.topic_raw, qos);
    pub_comp_ = create_publisher<sensor_msgs::msg::CompressedImage>(cfg_.topic_compressed, qos);
    pub_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(cfg_.topic_camera_info, qos);
    RCLCPP_INFO(get_logger(), "Raw: %s (%s), JPEG: %s (quality %d)", pub_raw_->get_topic_name(),
                cfg_.raw_mono ? "mono8" : "color", pub_comp_->get_topic_name(), cfg_.jpeg_quality);

    Driver::Callbacks cb;
    cb.log = log;
    cb.want_image = [this] {
      return pub_raw_->get_subscription_count() > 0 || pub_comp_->get_subscription_count() > 0;
    };
    cb.publish = [this](const Frame &f) { publish(f); };
    driver_.reset(new Driver(cfg_, cb));
    stats_timer_ = create_wall_timer(std::chrono::seconds(10), [this] {
      RCLCPP_INFO(get_logger(), "10 s: %s", driver_->statsLine().c_str());
    });
  }

  // CameraInfoManager needs shared_from_this, so it is created after construction.
  // Service <camera_name>/set_camera_info as in ROS 1 (camera_calibration with camera:=/<camera_name>)
  bool start() {
    cim_node_ = create_sub_node(cfg_.camera_name);
    cim_ = std::make_shared<camera_info_manager::CameraInfoManager>(cim_node_.get(), cfg_.camera_name,
                                                                    cfg_.calibration_url);
    return driver_->start();
  }
  void stop() { driver_->stop(); }

 private:
  void publish(const Frame &f) {
    const rclcpp::Time stamp(f.stamp_ns, RCL_SYSTEM_TIME);
    if (!f.image.empty()) {
      const int w = f.image.cols, h = f.image.rows;
      if (pub_raw_->get_subscription_count() > 0) {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
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
        pub_raw_->publish(std::move(msg));
      }
      if (pub_comp_->get_subscription_count() > 0) {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = stamp;
        msg->header.frame_id = cfg_.frame_id;
        msg->format = "jpeg";
        if (toJpeg(f, cfg_.jpeg_quality, msg->data)) pub_comp_->publish(std::move(msg));
      }
      last_w_ = w;
      last_h_ = h;
    }
    auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cim_->getCameraInfo());
    ci->header.stamp = stamp;
    ci->header.frame_id = cfg_.frame_id;
    if (last_w_ > 0) {
      ci->width = last_w_;
      ci->height = last_h_;
    }
    pub_info_->publish(std::move(ci));
  }

  Config cfg_;
  rclcpp::Node::SharedPtr cim_node_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cim_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_comp_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::unique_ptr<Driver> driver_;
  int last_w_ = 0, last_h_ = 0;  // only used in the publish thread
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ros2Node>();
  if (!node->start()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  node->stop();
  rclcpp::shutdown();
  return 0;
}
