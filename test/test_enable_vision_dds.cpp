#include <memory>

#include <gtest/gtest.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "vision_to_dds/vision_to_dds.hpp"

// This fixture is a friend of VisionToDDS so the fault-path test can drive a
// single timer cycle without starting the node's blocking run() method.
class VisionToDDSOutputTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  static std::shared_ptr<VisionToDDS> makeNode(bool enable_vision_dds)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("enable_vision_dds", enable_vision_dds)});
    return std::make_shared<VisionToDDS>(options);
  }

  static void runOneOutputCycle(VisionToDDS & node)
  {
    node.publishVisionPositionEstimate();
  }
};

TEST_F(VisionToDDSOutputTest, DefaultConfigurationCreatesNoVisionDdsWriter)
{
  const auto node = std::make_shared<VisionToDDS>();

  EXPECT_FALSE(node->visionDdsEnabled());
  EXPECT_FALSE(node->hasVehicleOdometryPublisher());
  EXPECT_FALSE(node->mayPublishVisionDds());
}

TEST_F(VisionToDDSOutputTest, ExplicitEnableCreatesExactlyOneVisionDdsWriter)
{
  const auto node = makeNode(true);

  EXPECT_TRUE(node->visionDdsEnabled());
  EXPECT_TRUE(node->hasVehicleOdometryPublisher());
  EXPECT_EQ(node->targetTopicPublisherCount(), 1U);
  EXPECT_TRUE(node->mayPublishVisionDds());
}

TEST_F(VisionToDDSOutputTest, DuplicateWriterFaultStopsVisionDdsOutput)
{
  const auto node = makeNode(true);
  const auto duplicate_node = std::make_shared<rclcpp::Node>("vision_odometry_duplicate_writer");
  const auto duplicate_writer = duplicate_node->create_publisher<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry", rclcpp::QoS(1));
  (void)duplicate_writer;

  ASSERT_EQ(node->targetTopicPublisherCount(), 2U);
  runOneOutputCycle(*node);
  EXPECT_EQ(node->visionDdsFault(), vision_to_dds::FaultCode::kDuplicateWriter);
  EXPECT_FALSE(node->mayPublishVisionDds());
}
