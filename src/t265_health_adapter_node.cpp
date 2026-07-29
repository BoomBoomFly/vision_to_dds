#include "vision_to_dds/t265_health_adapter.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int32.hpp>

namespace
{
constexpr uint64_t kUsecPerSec = 1000000ULL;
constexpr uint64_t kNsecPerUsec = 1000ULL;

uint64_t secondsToUsec(double seconds, const char * parameter_name)
{
  if (!(seconds > 0.0) || seconds > 3600.0) {
    throw std::runtime_error(std::string(parameter_name) + " must be in (0, 3600]");
  }
  return static_cast<uint64_t>(seconds * static_cast<double>(kUsecPerSec));
}

class T265HealthAdapterNode : public rclcpp::Node
{
public:
  T265HealthAdapterNode()
  : Node("t265_health_adapter_node"), adapter_(parameters())
  {
    rclcpp::QoS health_qos(rclcpp::KeepLast(1));
    health_qos.best_effort().durability_volatile();
    quality_publisher_ = create_publisher<std_msgs::msg::Int8>(quality_topic_, health_qos);
    epoch_publisher_ = create_publisher<std_msgs::msg::UInt32>(source_epoch_topic_, health_qos);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, health_qos,
      std::bind(&T265HealthAdapterNode::odometryCallback, this, std::placeholders::_1));
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / health_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&T265HealthAdapterNode::timerCallback, this));
    publish(adapter_.tick(nowUsec()));
  }

private:
  vision_to_dds::T265HealthConfig parameters()
  {
    // This workstation launches the T265 under camera_name=t265.  Keep the
    // health adapter on the observed aircraft topic instead of the upstream
    // wrapper's generic camera namespace.
    declare_parameter<std::string>("odometry_topic", "/t265/pose/sample");
    declare_parameter<std::string>("quality_topic", "/vision/quality");
    declare_parameter<std::string>("source_epoch_topic", "/vision/source_epoch");
    declare_parameter<double>("stream_timeout_s", 0.20);
    declare_parameter<double>("linear_accel_covariance", 0.01);
    declare_parameter<double>("health_rate_hz", 20.0);
    get_parameter("odometry_topic", odometry_topic_);
    get_parameter("quality_topic", quality_topic_);
    get_parameter("source_epoch_topic", source_epoch_topic_);
    double stream_timeout_s = 0.0;
    get_parameter("stream_timeout_s", stream_timeout_s);
    get_parameter("linear_accel_covariance", linear_accel_covariance_);
    get_parameter("health_rate_hz", health_rate_hz_);
    if (odometry_topic_.empty() || quality_topic_.empty() || source_epoch_topic_.empty() ||
      !(health_rate_hz_ > 0.0) || health_rate_hz_ > 1000.0) {
      throw std::runtime_error("invalid T265 health adapter parameter");
    }
    vision_to_dds::T265HealthConfig config;
    config.stream_timeout_us = secondsToUsec(stream_timeout_s, "stream_timeout_s");
    config.linear_accel_covariance = linear_accel_covariance_;
    return config;
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    vision_to_dds::T265OdometryHealthSample sample;
    sample.timestamp_us = static_cast<uint64_t>(message->header.stamp.sec) * kUsecPerSec +
      static_cast<uint64_t>(message->header.stamp.nanosec) / kNsecPerUsec;
    sample.pose_covariance = message->pose.covariance;
    publish(adapter_.observeOdometry(sample, nowUsec()));
  }

  void timerCallback()
  {
    // Re-publish the measured state so the bridge can enforce quality
    // freshness.  Quality is never a fixed health constant.
    publish(adapter_.tick(nowUsec()));
  }

  void publish(const vision_to_dds::T265HealthOutput & status)
  {
    std_msgs::msg::UInt32 epoch;
    epoch.data = status.source_epoch;
    epoch_publisher_->publish(epoch);
    std_msgs::msg::Int8 quality;
    quality.data = status.quality;
    quality_publisher_->publish(quality);
  }

  uint64_t nowUsec()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / static_cast<int64_t>(kNsecPerUsec));
  }

  std::string odometry_topic_;
  std::string quality_topic_;
  std::string source_epoch_topic_;
  double linear_accel_covariance_{0.01};
  double health_rate_hz_{20.0};
  vision_to_dds::T265HealthAdapter adapter_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr quality_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr epoch_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<T265HealthAdapterNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("t265_health_adapter_node"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
