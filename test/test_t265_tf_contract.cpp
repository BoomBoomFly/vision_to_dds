#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "vision_to_dds/vision_contract.hpp"

namespace
{
using vision_to_dds::ContractConfig;
using vision_to_dds::Decision;
using vision_to_dds::FaultCode;
using vision_to_dds::TransformSample;
using vision_to_dds::VisionContract;

constexpr uint64_t kFirstStampUs = 1000000U;
constexpr uint64_t kSecondStampUs = 1100000U;

ContractConfig t265Config()
{
  ContractConfig config;
  config.world_frame_id = "odom_frame";
  config.body_frame_id = "base_link";
  return config;
}

TransformSample transform(
  uint64_t stamp_us, const tf2::Vector3 & position,
  const tf2::Quaternion & orientation = tf2::Quaternion(0.0, 0.0, 0.0, 1.0))
{
  TransformSample sample;
  sample.world_frame_id = "odom_frame";
  sample.body_frame_id = "base_link";
  sample.timestamp_us = stamp_us;
  sample.position_enu = position;
  sample.orientation_enu_flu = orientation;
  return sample;
}

void makeHealthy(VisionContract & contract, uint64_t now_us)
{
  contract.observeSourceEpoch(17U, now_us);
  contract.observeQuality(90, now_us);
}

vision_to_dds::VehicleOdometrySample converted(
  const tf2::Vector3 & position, const tf2::Quaternion & orientation)
{
  VisionContract contract(t265Config());
  makeHealthy(contract, kFirstStampUs);
  EXPECT_EQ(contract.evaluate(transform(kFirstStampUs, position, orientation), kFirstStampUs).decision,
    Decision::kWarmup);
  const auto result = contract.evaluate(transform(kSecondStampUs, position, orientation), kSecondStampUs);
  EXPECT_EQ(result.decision, Decision::kPublish);
  return result.odometry;
}

void expectEquivalentQuaternion(const tf2::Quaternion & actual, const tf2::Quaternion & expected)
{
  const double dot = actual.dot(expected);
  EXPECT_NEAR(std::fabs(dot), 1.0, 1e-12);
}

TEST(T265TfContract, OriginForwardLeftRightAndUpMapToExpectedNedAxes)
{
  const auto origin = converted(tf2::Vector3(0.0, 0.0, 0.0), tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(origin.position_ned.getX(), 0.0);
  EXPECT_DOUBLE_EQ(origin.position_ned.getY(), 0.0);
  EXPECT_DOUBLE_EQ(origin.position_ned.getZ(), 0.0);

  // In ENU, +X is east; PX4 NED represents east as +Y.
  const auto forward_x = converted(tf2::Vector3(1.0, 0.0, 0.0), tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(forward_x.position_ned.getX(), 0.0);
  EXPECT_DOUBLE_EQ(forward_x.position_ned.getY(), 1.0);
  EXPECT_DOUBLE_EQ(forward_x.position_ned.getZ(), 0.0);

  const auto left_y = converted(tf2::Vector3(0.0, 1.0, 0.0), tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(left_y.position_ned.getX(), 1.0);
  EXPECT_DOUBLE_EQ(left_y.position_ned.getY(), 0.0);
  EXPECT_DOUBLE_EQ(left_y.position_ned.getZ(), 0.0);

  const auto right_y = converted(tf2::Vector3(0.0, -1.0, 0.0), tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(right_y.position_ned.getX(), -1.0);
  EXPECT_DOUBLE_EQ(right_y.position_ned.getY(), 0.0);
  EXPECT_DOUBLE_EQ(right_y.position_ned.getZ(), 0.0);

  const auto up_z = converted(tf2::Vector3(0.0, 0.0, 1.0), tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(up_z.position_ned.getX(), 0.0);
  EXPECT_DOUBLE_EQ(up_z.position_ned.getY(), 0.0);
  EXPECT_DOUBLE_EQ(up_z.position_ned.getZ(), -1.0);
}

TEST(T265TfContract, IdentityAndPlusMinusNinetyDegreeYawMapToNedFrd)
{
  const double half_root = std::sqrt(0.5);
  const auto identity = converted(tf2::Vector3(0.0, 0.0, 0.0), tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  expectEquivalentQuaternion(identity.orientation_ned_frd, tf2::Quaternion(0.0, 0.0, -half_root, -half_root));

  // +90 degrees ENU yaw faces north, which is yaw 0 in NED.
  const auto yaw_plus = converted(
    tf2::Vector3(0.0, 0.0, 0.0), tf2::Quaternion(0.0, 0.0, half_root, half_root));
  expectEquivalentQuaternion(yaw_plus.orientation_ned_frd, tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  // -90 degrees ENU yaw faces south, which is 180 degrees yaw in NED.
  const auto yaw_minus = converted(
    tf2::Vector3(0.0, 0.0, 0.0), tf2::Quaternion(0.0, 0.0, -half_root, half_root));
  expectEquivalentQuaternion(yaw_minus.orientation_ned_frd, tf2::Quaternion(0.0, 0.0, 1.0, 0.0));
}

TEST(T265TfContract, RejectsNonUnitQuaternionAndNonFinitePositionOrOrientation)
{
  VisionContract nonunit_contract(t265Config());
  makeHealthy(nonunit_contract, kFirstStampUs);
  EXPECT_EQ(
    nonunit_contract.evaluate(
      transform(kFirstStampUs, tf2::Vector3(0.0, 0.0, 0.0), tf2::Quaternion(0.0, 0.0, 0.0, 2.0)),
      kFirstStampUs).fault,
    FaultCode::kInvalidSample);

  VisionContract infinity_contract(t265Config());
  makeHealthy(infinity_contract, kFirstStampUs);
  EXPECT_EQ(
    infinity_contract.evaluate(
      transform(kFirstStampUs, tf2::Vector3(std::numeric_limits<double>::infinity(), 0.0, 0.0)),
      kFirstStampUs).fault,
    FaultCode::kInvalidSample);

  VisionContract nan_contract(t265Config());
  makeHealthy(nan_contract, kFirstStampUs);
  EXPECT_EQ(
    nan_contract.evaluate(
      transform(kFirstStampUs, tf2::Vector3(0.0, 0.0, 0.0),
        tf2::Quaternion(0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 1.0)),
      kFirstStampUs).fault,
    FaultCode::kInvalidSample);
}

TEST(T265TfContract, TimestampRollbackAndSourceEpochChangeLatchFaults)
{
  VisionContract timestamp_contract(t265Config());
  makeHealthy(timestamp_contract, kFirstStampUs);
  EXPECT_EQ(timestamp_contract.evaluate(
    transform(kFirstStampUs, tf2::Vector3(0.0, 0.0, 0.0)), kFirstStampUs).decision, Decision::kWarmup);
  EXPECT_EQ(timestamp_contract.evaluate(
    transform(kFirstStampUs - 1U, tf2::Vector3(0.0, 0.0, 0.0)), kFirstStampUs).fault,
    FaultCode::kTimestampRollback);

  VisionContract epoch_contract(t265Config());
  makeHealthy(epoch_contract, kFirstStampUs);
  epoch_contract.observeSourceEpoch(18U, kFirstStampUs + 1U);
  EXPECT_EQ(epoch_contract.fault(), FaultCode::kSourceEpochChanged);
  EXPECT_FALSE(epoch_contract.writerEnabled());
}

TEST(T265TfContract, TopicPublisherCardinalityMustBeExactlyOne)
{
  VisionContract contract(t265Config());
  makeHealthy(contract, kFirstStampUs);
  contract.observeWriterCardinality(0U, kFirstStampUs);
  EXPECT_EQ(contract.fault(), FaultCode::kDuplicateWriter);
  EXPECT_FALSE(contract.writerEnabled());
}

}  // namespace
