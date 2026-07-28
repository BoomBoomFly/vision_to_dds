#ifndef VISION_TO_DDS__VISION_TO_DDS_HPP_
#define VISION_TO_DDS__VISION_TO_DDS_HPP_

#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vision_to_dds/vision_contract.hpp"

class VisionToDDS : public rclcpp::Node {
public:
  VisionToDDS();
  ~VisionToDDS() = default;
  void run();

private:
  void navigationParameters();
  void publishVisionPositionEstimate();
  void qualityCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void sourceEpochCallback(const std_msgs::msg::UInt32::SharedPtr msg);
  void resetFaultCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void emitFaultIfNew();
  static uint64_t toUsec(const rclcpp::Time & stamp);
  static uint64_t toUsec(const builtin_interfaces::msg::Time & stamp);

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_path_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr quality_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr source_epoch_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path body_path_;
  vision_to_dds::BoundedHistory<geometry_msgs::msg::PoseStamped> body_path_history_{200};
  vision_to_dds::VisionContract contract_;
  vision_to_dds::FaultCode last_reported_fault_{vision_to_dds::FaultCode::kNone};
  std::string world_frame_id_;
  std::string body_frame_id_;
  std::string vehicle_visual_odometry_topic_;
  std::string quality_topic_;
  std::string source_epoch_topic_;
  double output_rate_{20.0};
  std::size_t max_path_samples_{200};
};

#endif  // VISION_TO_DDS__VISION_TO_DDS_HPP_
