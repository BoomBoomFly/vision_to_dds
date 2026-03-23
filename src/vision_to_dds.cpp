#include <vision_to_dds/vision_to_dds.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

constexpr double kInitialTransformTimeoutSec = 12.0;
constexpr double kTransformCheckRateHz = 3.0;
constexpr double kCanTransformTimeoutSec = 3.0;
constexpr uint64_t kUsecPerSec = 1000000ULL;
constexpr uint64_t kNsecPerUsec = 1000ULL;

inline uint64_t toUsec(const builtin_interfaces::msg::Time &stamp)
{
	return static_cast<uint64_t>(stamp.sec) * kUsecPerSec +
		static_cast<uint64_t>(stamp.nanosec) / kNsecPerUsec;
}

inline tf2::Vector3 rotateWorldToBody(const tf2::Vector3 &world, double gamma_world)
{
	tf2::Vector3 body;
	body.setX(std::cos(gamma_world) * world.getX() + std::sin(gamma_world) * world.getY());
	body.setY(-std::sin(gamma_world) * world.getX() + std::cos(gamma_world) * world.getY());
	body.setZ(world.getZ());
	return body;
}

}  // namespace

/**
 * 节点构造：
 * 1) 创建 TF Buffer 和 Listener
 * 2) 加入 ROS Timer 接口（Foxy 下推荐）
 * 3) 读取导航与精降参数并创建发布器
 */
VisionToDDS::VisionToDDS() : Node("vision_to_dds_node") {
	buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

	auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
		this->get_node_base_interface(), this->get_node_timers_interface());
	buffer_->setCreateTimerInterface(timer_interface);

	transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

	navigationParameters();
	precisionLandParameters();
}

// 运行主入口：等待首帧 TF -> 启动定时器循环 -> spin。
void VisionToDDS::run() {
	RCLCPP_INFO(this->get_logger(), "Running Vision To DDS (ROS2 bridge mode)");

	if (!waitForFirstTransform(kInitialTransformTimeoutSec)) {
		return;
	}

	RCLCPP_INFO(this->get_logger(), "First transform is received");
	last_tf_time_ = this->get_clock()->now();

	timer_ = this->create_wall_timer(
		std::chrono::milliseconds(static_cast<int>(1000.0 / output_rate_)),
		std::bind(&VisionToDDS::publishVisionPositionEstimate, this));

	rclcpp::spin(this->shared_from_this());
}

/**
 * 导航相关参数和发布器初始化：
 * - 调试可视化话题：vision_pose / body_frame/path
 * - PX4 输入话题：/fmu/in/vehicle_visual_odometry
 * - 坐标系变换参数：gamma_world / roll_cam / pitch_cam / yaw_cam
 */
void VisionToDDS::navigationParameters() {
	this->declare_parameter<std::string>(
		"vehicle_visual_odometry_topic", "/fmu/in/vehicle_visual_odometry");
	this->get_parameter("vehicle_visual_odometry_topic", vehicle_visual_odometry_topic_);

	vehicle_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
		vehicle_visual_odometry_topic_, 10);

	camera_pose_publisher_ =
		this->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose", 10);
	body_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);

	this->declare_parameter<std::string>("target_frame_id", "/camera_odom_frame");
	this->get_parameter("target_frame_id", target_frame_id_);

	this->declare_parameter<std::string>("source_frame_id", "/camera_link");
	this->get_parameter("source_frame_id", source_frame_id_);

	this->declare_parameter<double>("output_rate", 20.0);
	this->get_parameter("output_rate", output_rate_);

	this->declare_parameter<double>("gamma_world", -1.5707963);
	this->get_parameter("gamma_world", gamma_world_);

	this->declare_parameter<double>("roll_cam", 0.0);
	this->get_parameter("roll_cam", roll_cam_);

	this->declare_parameter<double>("pitch_cam", 0.0);
	this->get_parameter("pitch_cam", pitch_cam_);

	this->declare_parameter<double>("yaw_cam", 1.5707963);
	this->get_parameter("yaw_cam", yaw_cam_);

	RCLCPP_INFO(this->get_logger(), "Get target_frame_id: %s", target_frame_id_.c_str());
	RCLCPP_INFO(this->get_logger(), "Get source_frame_id: %s", source_frame_id_.c_str());
	RCLCPP_INFO(
		this->get_logger(),
		"Get vehicle_visual_odometry_topic: %s",
		vehicle_visual_odometry_topic_.c_str());
	RCLCPP_INFO(this->get_logger(), "Get output_rate: %f", output_rate_);
	RCLCPP_INFO(this->get_logger(), "Get gamma_world: %f", gamma_world_);
	RCLCPP_INFO(this->get_logger(), "Get roll_cam: %f", roll_cam_);
	RCLCPP_INFO(this->get_logger(), "Get pitch_cam: %f", pitch_cam_);
	RCLCPP_INFO(this->get_logger(), "Get yaw_cam: %f", yaw_cam_);
}

/**
 * 精准降落参数初始化：
 * - enable_precland 关闭时不创建精降发布器
 * - 开启后发布 px4_msgs::LandingTargetPose 到 /fmu/in/landing_target_pose
 */
void VisionToDDS::precisionLandParameters() {
	this->declare_parameter<bool>("enable_precland", false);
	this->get_parameter("enable_precland", enable_precland_);

	RCLCPP_INFO(
		this->get_logger(),
		"Precision landing: %s",
		enable_precland_ ? "enabled" : "disabled");

	if (enable_precland_) {
		this->declare_parameter<std::string>("landing_target_pose_topic", "/fmu/in/landing_target_pose");
		this->get_parameter("landing_target_pose_topic", landing_target_pose_topic_);

		this->declare_parameter<std::string>("precland_target_frame_id", "/landing_target");
		this->get_parameter("precland_target_frame_id", precland_target_frame_id_);
		this->declare_parameter<std::string>(
			"precland_camera_frame_id", "/camera_fisheye2_optical_frame");
		this->get_parameter("precland_camera_frame_id", precland_camera_frame_id_);

		RCLCPP_INFO(
			this->get_logger(),
			"Get precland_target_frame_id: %s",
			precland_target_frame_id_.c_str());
		RCLCPP_INFO(
			this->get_logger(),
			"Get precland_camera_frame_id: %s",
			precland_camera_frame_id_.c_str());
		RCLCPP_INFO(
			this->get_logger(),
			"Get landing_target_pose_topic: %s",
			landing_target_pose_topic_.c_str());

		landing_target_pose_publisher_ = this->create_publisher<px4_msgs::msg::LandingTargetPose>(
			landing_target_pose_topic_, 10);
	}
}

// 等待首帧 TF，确保主循环运行时有可用位姿参考。
bool VisionToDDS::waitForFirstTransform(double timeout) {
	bool received = false;
	std::string error_msg;
	const auto start_time = this->now();

	RCLCPP_INFO(
		this->get_logger(),
		"Waiting for transform between %s and %s",
		target_frame_id_.c_str(),
		source_frame_id_.c_str());

	rclcpp::Rate rate(kTransformCheckRateHz);
	while (rclcpp::ok() && (this->now() - start_time < rclcpp::Duration::from_seconds(timeout))) {
		if (buffer_->canTransform(
				target_frame_id_,
				source_frame_id_,
				this->get_clock()->now(),
				rclcpp::Duration::from_seconds(kCanTransformTimeoutSec),
				&error_msg)) {
			received = true;
			break;
		}

		RCLCPP_WARN(this->get_logger(), "Error message: %s", error_msg.c_str());
		RCLCPP_INFO(this->get_logger(), "Waiting for transform...");
		rate.sleep();
	}

	if (!received) {
		RCLCPP_ERROR(
			this->get_logger(), "Timeout waiting for transform after %.1f seconds.", timeout);
	}

	return received;
}

/**
 * 发布精降目标：
 * 1) 查询“目标在世界坐标”和“机体在世界坐标”
 * 2) 计算相对位置 rel_world = target - vehicle
 * 3) 旋转到 NED/body 对齐后填充 LandingTargetPose
 */
void VisionToDDS::publishPrecland(const builtin_interfaces::msg::Time &stamp) {
	if (!enable_precland_ || !landing_target_pose_publisher_) {
		return;
	}

	try {
		const auto target_in_world =
			buffer_->lookupTransform(target_frame_id_, precland_target_frame_id_, tf2::TimePointZero);
		const auto vehicle_in_world =
			buffer_->lookupTransform(target_frame_id_, source_frame_id_, tf2::TimePointZero);

		const auto &tt = target_in_world.transform.translation;
		const auto &tv = vehicle_in_world.transform.translation;

		const tf2::Vector3 target_world(tt.x, tt.y, tt.z);
		const tf2::Vector3 vehicle_world(tv.x, tv.y, tv.z);
		const tf2::Vector3 rel_world = target_world - vehicle_world;

		const tf2::Vector3 rel_ned = rotateWorldToBody(rel_world, gamma_world_);
		const tf2::Vector3 abs_ned = rotateWorldToBody(target_world, gamma_world_);

		px4_msgs::msg::LandingTargetPose msg;
		msg.timestamp = toUsec(stamp);
		msg.is_static = true;
		msg.rel_pos_valid = true;
		msg.rel_vel_valid = false;
		msg.x_rel = static_cast<float>(rel_ned.getX());
		msg.y_rel = static_cast<float>(rel_ned.getY());
		msg.z_rel = static_cast<float>(rel_ned.getZ());
		msg.vx_rel = 0.0F;
		msg.vy_rel = 0.0F;
		msg.cov_x_rel = 0.05F;
		msg.cov_y_rel = 0.05F;
		msg.cov_vx_rel = 0.1F;
		msg.cov_vy_rel = 0.1F;
		msg.abs_pos_valid = true;
		msg.x_abs = static_cast<float>(abs_ned.getX());
		msg.y_abs = static_cast<float>(abs_ned.getY());
		msg.z_abs = static_cast<float>(abs_ned.getZ());

		landing_target_pose_publisher_->publish(msg);
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN_THROTTLE(
			this->get_logger(), *this->get_clock(), 3000, "Precland tf error: %s", ex.what());
	}
}

/**
 * 核心循环：
 * - 读取 camera_link 相对 camera_odom_frame 的 TF
 * - 计算机体位姿（位置旋转 + 姿态补偿）
 * - 发布调试位姿、轨迹、PX4 VehicleOdometry、精降信息
 */
void VisionToDDS::publishVisionPositionEstimate() {
	const float nan_f = std::numeric_limits<float>::quiet_NaN();

	try {
		const auto transform_stamped =
			buffer_->lookupTransform(target_frame_id_, source_frame_id_, tf2::TimePointZero);

		if (last_tf_time_ < transform_stamped.header.stamp) {
			last_tf_time_ = transform_stamped.header.stamp;

			const auto &t = transform_stamped.transform.translation;
			const tf2::Vector3 position_orig(t.x, t.y, t.z);

			const tf2::Vector3 position_body = rotateWorldToBody(position_orig, gamma_world_);

			const auto &q = transform_stamped.transform.rotation;
			tf2::Quaternion quat_cam(q.x, q.y, q.z, q.w);

			tf2::Quaternion quat_cam_to_body_x;
			tf2::Quaternion quat_cam_to_body_y;
			tf2::Quaternion quat_cam_to_body_z;
			tf2::Quaternion quat_rot_z;

			quat_cam_to_body_x.setRPY(roll_cam_, 0.0, 0.0);
			quat_cam_to_body_y.setRPY(0.0, pitch_cam_, 0.0);
			quat_cam_to_body_z.setRPY(0.0, 0.0, yaw_cam_);
			quat_rot_z.setRPY(0.0, 0.0, -gamma_world_);

			tf2::Quaternion quat_body =
				quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
			quat_body.normalize();

			geometry_msgs::msg::PoseStamped msg_body_pose;
			msg_body_pose.header.stamp = transform_stamped.header.stamp;
			msg_body_pose.header.frame_id = transform_stamped.header.frame_id;
			msg_body_pose.pose.position.x = position_body.getX();
			msg_body_pose.pose.position.y = position_body.getY();
			msg_body_pose.pose.position.z = position_body.getZ();
			msg_body_pose.pose.orientation.x = quat_body.getX();
			msg_body_pose.pose.orientation.y = quat_body.getY();
			msg_body_pose.pose.orientation.z = quat_body.getZ();
			msg_body_pose.pose.orientation.w = quat_body.getW();

			camera_pose_publisher_->publish(msg_body_pose);

			px4_msgs::msg::VehicleOdometry msg_odom;
			msg_odom.timestamp =
				static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / kNsecPerUsec);
			msg_odom.timestamp_sample = toUsec(transform_stamped.header.stamp);
			msg_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
			msg_odom.position[0] = static_cast<float>(position_body.getX());
			msg_odom.position[1] = static_cast<float>(position_body.getY());
			msg_odom.position[2] = static_cast<float>(position_body.getZ());
			msg_odom.q[0] = static_cast<float>(quat_body.getW());
			msg_odom.q[1] = static_cast<float>(quat_body.getX());
			msg_odom.q[2] = static_cast<float>(quat_body.getY());
			msg_odom.q[3] = static_cast<float>(quat_body.getZ());
			msg_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
			msg_odom.velocity[0] = nan_f;
			msg_odom.velocity[1] = nan_f;
			msg_odom.velocity[2] = nan_f;
			msg_odom.angular_velocity[0] = nan_f;
			msg_odom.angular_velocity[1] = nan_f;
			msg_odom.angular_velocity[2] = nan_f;
			msg_odom.position_variance[0] = 0.02F;
			msg_odom.position_variance[1] = 0.02F;
			msg_odom.position_variance[2] = 0.04F;
			msg_odom.orientation_variance[0] = 0.01F;
			msg_odom.orientation_variance[1] = 0.01F;
			msg_odom.orientation_variance[2] = 0.02F;
			msg_odom.velocity_variance[0] = nan_f;
			msg_odom.velocity_variance[1] = nan_f;
			msg_odom.velocity_variance[2] = nan_f;
			msg_odom.reset_counter = 0;
			msg_odom.quality = 1;

			vehicle_odometry_publisher_->publish(msg_odom);

			body_path_.header.stamp = msg_body_pose.header.stamp;
			body_path_.header.frame_id = msg_body_pose.header.frame_id;
			body_path_.poses.push_back(msg_body_pose);
			body_path_publisher_->publish(body_path_);

			publishPrecland(msg_body_pose.header.stamp);
		}
	} catch (const tf2::TransformException &ex) {
		RCLCPP_WARN(this->get_logger(), "%s", ex.what());
		rclcpp::sleep_for(std::chrono::seconds(1));
	}
}

// 程序入口：初始化 ROS -> 运行节点 -> 退出清理。
int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<VisionToDDS>();
	node->run();
	rclcpp::shutdown();
	return 0;
}
