#ifndef VISION_TO_DDS_H
#define VISION_TO_DDS_H

#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/landing_target_pose.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

class VisionToDDS : public rclcpp::Node {
public:
	VisionToDDS();
	~VisionToDDS() = default;

	void run();

private:
	void navigationParameters();
	void precisionLandParameters();
	bool waitForFirstTransform(double timeout);
	void publishPrecland(const builtin_interfaces::msg::Time &stamp);
	void publishVisionPositionEstimate();

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_path_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_publisher_;
	rclcpp::Publisher<px4_msgs::msg::LandingTargetPose>::SharedPtr landing_target_pose_publisher_;

	std::shared_ptr<tf2_ros::Buffer> buffer_;
	std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
	rclcpp::TimerBase::SharedPtr timer_;

	nav_msgs::msg::Path body_path_;
	rclcpp::Time last_tf_time_;

	std::string target_frame_id_;
	std::string source_frame_id_;
	std::string vehicle_visual_odometry_topic_;
	std::string landing_target_pose_topic_;
	std::string precland_target_frame_id_;
	std::string precland_camera_frame_id_;

	double output_rate_{20.0};
	double gamma_world_{-1.5707963};
	double roll_cam_{0.0};
	double pitch_cam_{0.0};
	double yaw_cam_{1.5707963};
	bool enable_precland_{false};
};

#endif
