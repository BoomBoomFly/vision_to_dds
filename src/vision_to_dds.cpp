#include "vision_to_dds/vision_to_dds.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

std::array<float, 3> varianceParameter(
  rclcpp::Node & node, const std::string & name, const std::array<float, 3> & defaults)
{
  node.declare_parameter<std::vector<double>>(name, {defaults[0], defaults[1], defaults[2]});
  const std::vector<double> values = node.get_parameter(name).as_double_array();
  if (values.size() != 3U) {
    throw std::runtime_error(name + " must contain exactly three values");
  }
  std::array<float, 3> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = static_cast<float>(values[index]);
  }
  return result;
}
}  // namespace

VisionToDDS::VisionToDDS()
: VisionToDDS(rclcpp::NodeOptions())
{
}

VisionToDDS::VisionToDDS(const rclcpp::NodeOptions & options)
: Node("vision_to_dds_node", options), contract_(vision_to_dds::ContractConfig{})
{
  navigationParameters();
  // H0: with the production writer disabled, the node remains available for
  // process-level diagnostics but owns no TF transport endpoint.
  if (!enable_vision_dds_) {
    return;
  }
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
}

void VisionToDDS::run()
{
  // Bind TF subscriptions to this node executor after construction.
  // The main executor below services these TF callbacks.
  if (enable_vision_dds_ && !transform_listener_) {
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *buffer_, this->shared_from_this(), false);
  }
  RCLCPP_INFO(
    this->get_logger(),
    enable_vision_dds_ ?
    "vision odometry bridge starts fail-closed; awaiting source epoch, quality, and two fresh TF samples" :
    "vision odometry DDS output is disabled; diagnostics remain active");
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / output_rate_));
  timer_ = this->create_wall_timer(period, std::bind(&VisionToDDS::publishVisionPositionEstimate, this));
  rclcpp::spin(this->shared_from_this());
}

void VisionToDDS::navigationParameters()
{
  this->declare_parameter<std::string>("vehicle_visual_odometry_topic", "/fmu/in/vehicle_visual_odometry");
  this->get_parameter("vehicle_visual_odometry_topic", vehicle_visual_odometry_topic_);
  this->declare_parameter<bool>("enable_vision_dds", false);
  this->get_parameter("enable_vision_dds", enable_vision_dds_);
  this->declare_parameter<std::string>("world_frame_id", "odom_frame");
  this->get_parameter("world_frame_id", world_frame_id_);
  this->declare_parameter<std::string>("body_frame_id", "base_link");
  this->get_parameter("body_frame_id", body_frame_id_);
  this->declare_parameter<std::string>("quality_topic", "/vision/quality");
  this->get_parameter("quality_topic", quality_topic_);
  this->declare_parameter<std::string>("source_epoch_topic", "/vision/source_epoch");
  this->get_parameter("source_epoch_topic", source_epoch_topic_);
  this->declare_parameter<double>("output_rate", 20.0);
  this->get_parameter("output_rate", output_rate_);
  this->declare_parameter<int>("minimum_quality", 50);
  this->declare_parameter<double>("maximum_sample_age_s", 0.45);
  this->declare_parameter<double>("maximum_future_skew_s", 0.02);
  this->declare_parameter<double>("maximum_timestamp_jump_s", 0.50);
  this->declare_parameter<double>("quality_timeout_s", 0.25);
  this->declare_parameter<int>("max_path_samples", 200);

  int minimum_quality = 0;
  int max_path_samples = 0;
  double maximum_sample_age_s = 0.0;
  double maximum_future_skew_s = 0.0;
  double maximum_timestamp_jump_s = 0.0;
  double quality_timeout_s = 0.0;
  this->get_parameter("minimum_quality", minimum_quality);
  this->get_parameter("maximum_sample_age_s", maximum_sample_age_s);
  this->get_parameter("maximum_future_skew_s", maximum_future_skew_s);
  this->get_parameter("maximum_timestamp_jump_s", maximum_timestamp_jump_s);
  this->get_parameter("quality_timeout_s", quality_timeout_s);
  this->get_parameter("max_path_samples", max_path_samples);
  if (!(output_rate_ > 0.0) || output_rate_ > 1000.0 || minimum_quality < -128 || minimum_quality > 127 ||
    max_path_samples < 0 || world_frame_id_.empty() || body_frame_id_.empty() || quality_topic_.empty() ||
    source_epoch_topic_.empty()) {
    throw std::runtime_error("invalid vision fail-closed contract parameter");
  }

  vision_to_dds::ContractConfig config;
  config.world_frame_id = world_frame_id_;
  config.body_frame_id = body_frame_id_;
  config.minimum_quality = static_cast<int8_t>(minimum_quality);
  config.maximum_sample_age_us = secondsToUsec(maximum_sample_age_s, "maximum_sample_age_s");
  config.maximum_future_skew_us = secondsToUsec(maximum_future_skew_s, "maximum_future_skew_s");
  config.maximum_timestamp_jump_us = secondsToUsec(maximum_timestamp_jump_s, "maximum_timestamp_jump_s");
  config.quality_timeout_us = secondsToUsec(quality_timeout_s, "quality_timeout_s");
  config.position_variance_enu = varianceParameter(*this, "position_variance_enu", config.position_variance_enu);
  config.orientation_variance_flu = varianceParameter(*this, "orientation_variance_flu", config.orientation_variance_flu);
  config.velocity_variance_enu = varianceParameter(*this, "velocity_variance_enu", config.velocity_variance_enu);
  contract_ = vision_to_dds::VisionContract(config);
  max_path_samples_ = static_cast<std::size_t>(max_path_samples);
  body_path_history_.setCapacity(max_path_samples_);

  // H0 must precede every ROS publisher creation.  This gives the disabled
  // process no visual DDS writer (nor any accidental diagnostic writer).
  if (!enable_vision_dds_) {
    return;
  }
  rclcpp::QoS px4_qos(rclcpp::KeepLast(1));
  vehicle_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
    vehicle_visual_odometry_topic_, px4_qos.best_effort().durability_volatile());
  camera_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose", 10);
  body_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);
  fault_publisher_ = this->create_publisher<std_msgs::msg::String>("~/fault", 10);
  quality_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
    quality_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&VisionToDDS::qualityCallback, this, std::placeholders::_1));
  source_epoch_subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
    source_epoch_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&VisionToDDS::sourceEpochCallback, this, std::placeholders::_1));
  reset_fault_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset_fault", std::bind(
      &VisionToDDS::resetFaultCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void VisionToDDS::publishVisionPositionEstimate()
{
  // H0 interlock: no DDS endpoint is constructed when disabled, and the
  // periodic diagnostic work must never be able to emit VehicleOdometry.
  if (!enable_vision_dds_) {
    return;
  }

  const uint64_t now_us = toUsec(this->get_clock()->now());
  contract_.observeWriterCardinality(targetTopicPublisherCount(), now_us);
  contract_.tick(now_us);
  emitFaultIfNew();
  if (!mayPublishVisionDds()) {
    return;
  }

  try {
    const auto transform = buffer_->lookupTransform(world_frame_id_, body_frame_id_, tf2::TimePointZero);
    vision_to_dds::TransformSample input;
    input.world_frame_id = transform.header.frame_id;
    input.body_frame_id = transform.child_frame_id;
    input.timestamp_us = toUsec(transform.header.stamp);
    input.position_enu = tf2::Vector3(
      transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    input.orientation_enu_flu = tf2::Quaternion(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);
    const auto evaluation = contract_.evaluate(input, now_us);
    emitFaultIfNew();
    if (evaluation.decision != vision_to_dds::Decision::kPublish) {
      return;
    }

    px4_msgs::msg::VehicleOdometry odometry;
    odometry.timestamp = evaluation.odometry.timestamp_us;
    odometry.timestamp_sample = evaluation.odometry.timestamp_sample_us;
    odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odometry.position[0] = static_cast<float>(evaluation.odometry.position_ned.getX());
    odometry.position[1] = static_cast<float>(evaluation.odometry.position_ned.getY());
    odometry.position[2] = static_cast<float>(evaluation.odometry.position_ned.getZ());
    odometry.q[0] = static_cast<float>(evaluation.odometry.orientation_ned_frd.getW());
    odometry.q[1] = static_cast<float>(evaluation.odometry.orientation_ned_frd.getX());
    odometry.q[2] = static_cast<float>(evaluation.odometry.orientation_ned_frd.getY());
    odometry.q[3] = static_cast<float>(evaluation.odometry.orientation_ned_frd.getZ());
    odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    odometry.velocity[0] = static_cast<float>(evaluation.odometry.velocity_ned.getX());
    odometry.velocity[1] = static_cast<float>(evaluation.odometry.velocity_ned.getY());
    odometry.velocity[2] = static_cast<float>(evaluation.odometry.velocity_ned.getZ());
    odometry.angular_velocity[0] = 0.0F;
    odometry.angular_velocity[1] = 0.0F;
    odometry.angular_velocity[2] = 0.0F;
    for (std::size_t index = 0; index < 3; ++index) {
      odometry.position_variance[index] = evaluation.odometry.position_variance_ned[index];
      odometry.orientation_variance[index] = evaluation.odometry.orientation_variance_frd[index];
      odometry.velocity_variance[index] = evaluation.odometry.velocity_variance_ned[index];
    }
    odometry.reset_counter = evaluation.odometry.reset_counter;
    odometry.quality = evaluation.odometry.quality;
    // The pointer is checked by mayPublishVisionDds() before entering the
    // contract path; retain the guard so a future refactor cannot accidentally
    // turn a disabled configuration into an output path.
    if (!vehicle_odometry_publisher_) {
      return;
    }
    vehicle_odometry_publisher_->publish(odometry);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.header.frame_id = "NED";
    pose.pose.position.x = evaluation.odometry.position_ned.getX();
    pose.pose.position.y = evaluation.odometry.position_ned.getY();
    pose.pose.position.z = evaluation.odometry.position_ned.getZ();
    pose.pose.orientation.x = evaluation.odometry.orientation_ned_frd.getX();
    pose.pose.orientation.y = evaluation.odometry.orientation_ned_frd.getY();
    pose.pose.orientation.z = evaluation.odometry.orientation_ned_frd.getZ();
    pose.pose.orientation.w = evaluation.odometry.orientation_ned_frd.getW();
    camera_pose_publisher_->publish(pose);
    if (max_path_samples_ > 0U) {
      body_path_.header = pose.header;
      body_path_history_.push(pose);
      body_path_.poses.assign(body_path_history_.values().begin(), body_path_history_.values().end());
      body_path_publisher_->publish(body_path_);
    }
  } catch (const tf2::TransformException &) {
    // A missing transform is handled by tick() once the last fresh sample ages out.
  }
}

std::size_t VisionToDDS::targetTopicPublisherCount() const
{
  // ROS 2 Foxy's graph API reports endpoints for the topic itself.  Counting
  // node names is not a writer cardinality check: a node may own zero or many
  // publishers on this topic.
  return this->get_publishers_info_by_topic(vehicle_visual_odometry_topic_).size();
}

void VisionToDDS::qualityCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  contract_.observeQuality(msg->data, toUsec(this->get_clock()->now()));
  emitFaultIfNew();
}

void VisionToDDS::sourceEpochCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
  contract_.observeSourceEpoch(msg->data, toUsec(this->get_clock()->now()));
  emitFaultIfNew();
}

void VisionToDDS::resetFaultCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  contract_.resetFault();
  last_reported_fault_ = vision_to_dds::FaultCode::kNone;
  response->success = contract_.writerEnabled();
  response->message = response->success ? "fault reset; bridge remains in warmup until health and fresh samples arrive" :
    "fault reset rejected by invalid configuration";
}

void VisionToDDS::emitFaultIfNew()
{
  const auto fault = contract_.fault();
  if (fault == vision_to_dds::FaultCode::kNone || fault == last_reported_fault_) {
    return;
  }
  last_reported_fault_ = fault;
  std_msgs::msg::String event;
  std::ostringstream json;
  json << "{\"component\":\"vision_to_dds\",\"event\":\"fault\",\"code\":\""
       << vision_to_dds::faultCodeName(fault) << "\",\"writer\":\"stopped\"}";
  event.data = json.str();
  fault_publisher_->publish(event);
  RCLCPP_ERROR(this->get_logger(), "%s", event.data.c_str());
}

uint64_t VisionToDDS::toUsec(const rclcpp::Time & stamp)
{
  return static_cast<uint64_t>(stamp.nanoseconds() / static_cast<int64_t>(kNsecPerUsec));
}

uint64_t VisionToDDS::toUsec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint64_t>(stamp.sec) * kUsecPerSec +
    static_cast<uint64_t>(stamp.nanosec) / kNsecPerUsec;
}

#ifndef VISION_TO_DDS_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VisionToDDS>();
    node->run();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("vision_to_dds_node"), "fatal configuration error: %s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
#endif  // VISION_TO_DDS_NO_MAIN
