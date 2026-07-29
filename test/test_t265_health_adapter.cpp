#include <gtest/gtest.h>

#include "vision_to_dds/t265_health_adapter.hpp"
#include "vision_to_dds/vision_contract.hpp"

namespace
{
using vision_to_dds::Decision;
using vision_to_dds::T265HealthAdapter;
using vision_to_dds::T265OdometryHealthSample;
using vision_to_dds::TransformSample;
using vision_to_dds::VisionContract;

T265OdometryHealthSample odometry(uint64_t timestamp_us, double variance)
{
  T265OdometryHealthSample sample;
  sample.timestamp_us = timestamp_us;
  sample.pose_covariance[0] = variance;
  sample.pose_covariance[7] = variance;
  sample.pose_covariance[14] = variance;
  return sample;
}

TransformSample transform(uint64_t timestamp_us)
{
  TransformSample sample;
  sample.world_frame_id = "odom_frame";
  sample.body_frame_id = "base_link";
  sample.timestamp_us = timestamp_us;
  sample.position_enu = tf2::Vector3(static_cast<double>(timestamp_us) / 1000000.0, 0.0, 0.0);
  sample.orientation_enu_flu = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  return sample;
}

TEST(T265HealthAdapter, TrackingNormalMapsMeasuredHighConfidence)
{
  T265HealthAdapter adapter(vision_to_dds::T265HealthConfig{});
  const auto result = adapter.observeOdometry(odometry(1000000U, 0.01), 1000000U);
  EXPECT_EQ(result.source_epoch, 1U);
  EXPECT_EQ(result.quality, 100);
}

TEST(T265HealthAdapter, TrackingLowAndFailedMapToMeasuredLowQuality)
{
  T265HealthAdapter adapter(vision_to_dds::T265HealthConfig{});
  EXPECT_EQ(adapter.observeOdometry(odometry(1000000U, 1.0), 1000000U).quality, 33);
  EXPECT_EQ(adapter.observeOdometry(odometry(1100000U, 10.0), 1100000U).quality, 0);
}

TEST(T265HealthAdapter, FreezeImmediatelyPublishesInvalidQualityAtDeadline)
{
  T265HealthAdapter adapter(vision_to_dds::T265HealthConfig{});
  adapter.observeOdometry(odometry(1000000U, 0.01), 1000000U);
  EXPECT_EQ(adapter.tick(1200001U).quality, 0);
}

TEST(T265HealthAdapter, ReconnectAfterFreezeAdvancesSourceEpoch)
{
  T265HealthAdapter adapter(vision_to_dds::T265HealthConfig{});
  adapter.observeOdometry(odometry(1000000U, 0.01), 1000000U);
  adapter.tick(1200001U);
  const auto recovered = adapter.observeOdometry(odometry(1300000U, 0.01), 1300000U);
  EXPECT_TRUE(recovered.source_epoch_changed);
  EXPECT_EQ(recovered.source_epoch, 2U);
  EXPECT_EQ(recovered.quality, 100);
}

TEST(T265HealthAdapter, TimestampRollbackAdvancesEpochAndInvalidatesImmediately)
{
  T265HealthAdapter adapter(vision_to_dds::T265HealthConfig{});
  adapter.observeOdometry(odometry(1000000U, 0.01), 1000000U);
  const auto rollback = adapter.observeOdometry(odometry(999999U, 0.01), 1000001U);
  EXPECT_TRUE(rollback.source_epoch_changed);
  EXPECT_EQ(rollback.source_epoch, 2U);
  EXPECT_EQ(rollback.quality, 0);
}

TEST(T265HealthAdapter, NoMessageTimeoutIsInvalidWithoutFabricatedQuality)
{
  T265HealthAdapter adapter(vision_to_dds::T265HealthConfig{});
  EXPECT_EQ(adapter.tick(250000U).quality, 0);
}

TEST(T265HealthAdapter, RecoveryRequiresBridgeResetAndFullTwoTransformWarmup)
{
  T265HealthAdapter adapter(vision_to_dds::T265HealthConfig{});
  VisionContract bridge(vision_to_dds::ContractConfig{});
  const auto healthy = adapter.observeOdometry(odometry(1000000U, 0.01), 1000000U);
  bridge.observeSourceEpoch(healthy.source_epoch, 1000000U);
  bridge.observeQuality(healthy.quality, 1000000U);
  EXPECT_EQ(bridge.evaluate(transform(1000000U), 1000000U).decision, Decision::kWarmup);
  EXPECT_EQ(bridge.evaluate(transform(1100000U), 1100000U).decision, Decision::kPublish);

  const auto lost = adapter.tick(1300001U);
  bridge.observeQuality(lost.quality, 1300001U);
  EXPECT_FALSE(bridge.writerEnabled());
  const auto recovered = adapter.observeOdometry(odometry(1400000U, 0.01), 1400000U);
  bridge.observeSourceEpoch(recovered.source_epoch, 1400000U);
  bridge.observeQuality(recovered.quality, 1400000U);
  EXPECT_FALSE(bridge.writerEnabled());

  bridge.resetFault();
  bridge.observeQuality(recovered.quality, 1400000U);
  EXPECT_EQ(bridge.evaluate(transform(1400000U), 1400000U).decision, Decision::kWarmup);
  EXPECT_EQ(bridge.evaluate(transform(1500000U), 1500000U).decision, Decision::kPublish);
}

}  // namespace
