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

TransformSample sample(uint64_t timestamp_us, double x, double y, double z)
{
  TransformSample value;
  value.world_frame_id = "odom_frame";
  value.body_frame_id = "base_link";
  value.timestamp_us = timestamp_us;
  value.position_enu = tf2::Vector3(x, y, z);
  value.orientation_enu_flu = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  return value;
}

void makeHealthy(VisionContract & contract, uint64_t now_us, int8_t quality = 80)
{
  contract.observeSourceEpoch(7U, now_us);
  contract.observeQuality(quality, now_us);
}

TEST(VisionContract, GoldenEnuFluToNedFrdTransformsPositionVelocityAttitudeAndCovariance)
{
  ContractConfig config;
  config.position_variance_enu = {{4.0F, 9.0F, 16.0F}};
  config.orientation_variance_flu = {{1.0F, 4.0F, 9.0F}};
  config.velocity_variance_enu = {{0.25F, 1.0F, 2.25F}};
  VisionContract contract(config);
  makeHealthy(contract, 1000000U);
  EXPECT_EQ(contract.evaluate(sample(1000000U, 1.0, 2.0, 3.0), 1000000U).decision, Decision::kWarmup);
  const auto result = contract.evaluate(sample(1100000U, 1.1, 2.2, 3.3), 1100000U);
  ASSERT_EQ(result.decision, Decision::kPublish);
  EXPECT_DOUBLE_EQ(result.odometry.position_ned.getX(), 2.2);
  EXPECT_DOUBLE_EQ(result.odometry.position_ned.getY(), 1.1);
  EXPECT_DOUBLE_EQ(result.odometry.position_ned.getZ(), -3.3);
  EXPECT_DOUBLE_EQ(result.odometry.velocity_ned.getX(), 2.0);
  EXPECT_DOUBLE_EQ(result.odometry.velocity_ned.getY(), 1.0);
  EXPECT_DOUBLE_EQ(result.odometry.velocity_ned.getZ(), -3.0);
  EXPECT_NEAR(std::fabs(result.odometry.orientation_ned_frd.getZ()), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(std::fabs(result.odometry.orientation_ned_frd.getW()), std::sqrt(0.5), 1e-12);
  EXPECT_EQ(result.odometry.position_variance_ned, (std::array<float, 3>{{9.0F, 4.0F, 16.0F}}));
  EXPECT_EQ(result.odometry.orientation_variance_frd, (std::array<float, 3>{{4.0F, 1.0F, 9.0F}}));
  EXPECT_EQ(result.odometry.velocity_variance_ned, (std::array<float, 3>{{1.0F, 0.25F, 2.25F}}));
  EXPECT_EQ(result.odometry.timestamp_us, result.odometry.timestamp_sample_us);
  EXPECT_EQ(result.odometry.quality, 80);
}

TEST(VisionContract, RejectsNanAndFrameContractMismatch)
{
  VisionContract nan_contract(ContractConfig{});
  makeHealthy(nan_contract, 1000000U);
  auto invalid = sample(1000000U, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  EXPECT_EQ(nan_contract.evaluate(invalid, 1000000U).fault, FaultCode::kInvalidSample);
  EXPECT_FALSE(nan_contract.writerEnabled());

  VisionContract frame_contract(ContractConfig{});
  makeHealthy(frame_contract, 1000000U);
  auto mismatch = sample(1000000U, 0.0, 0.0, 0.0);
  mismatch.body_frame_id = "unexpected_body";
  EXPECT_EQ(frame_contract.evaluate(mismatch, 1000000U).fault, FaultCode::kFrameMismatch);
}

TEST(VisionContract, LatchesClockTimestampAndSourceRestartFaults)
{
  VisionContract clock_contract(ContractConfig{});
  makeHealthy(clock_contract, 1000000U);
  EXPECT_EQ(clock_contract.tick(999999U).fault, FaultCode::kClockRollback);

  VisionContract timestamp_contract(ContractConfig{});
  makeHealthy(timestamp_contract, 1000000U);
  timestamp_contract.evaluate(sample(1000000U, 0.0, 0.0, 0.0), 1000000U);
  EXPECT_EQ(timestamp_contract.evaluate(sample(999999U, 0.0, 0.0, 0.0), 1000000U).fault,
    FaultCode::kTimestampRollback);

  VisionContract restart_contract(ContractConfig{});
  makeHealthy(restart_contract, 1000000U);
  restart_contract.observeSourceEpoch(8U, 1000001U);
  EXPECT_EQ(restart_contract.fault(), FaultCode::kSourceEpochChanged);
  restart_contract.observeQuality(90, 1000002U);
  EXPECT_FALSE(restart_contract.writerEnabled()) << "healthy data must not auto-recover ACTIVE";
  restart_contract.resetFault();
  EXPECT_TRUE(restart_contract.writerEnabled());
  EXPECT_EQ(restart_contract.resetCounter(), 1U);
}

TEST(VisionContract, FreezeQualityTimeoutAndTimestampJumpStopTheWriter)
{
  VisionContract freeze_contract(ContractConfig{});
  makeHealthy(freeze_contract, 1000000U);
  freeze_contract.evaluate(sample(1000000U, 0.0, 0.0, 0.0), 1000000U);
  EXPECT_EQ(freeze_contract.tick(1200001U).fault, FaultCode::kInputTimeout);

  VisionContract quality_contract(ContractConfig{});
  makeHealthy(quality_contract, 1000000U, 10);
  EXPECT_EQ(quality_contract.fault(), FaultCode::kQualityLow);

  ContractConfig jump_config;
  jump_config.maximum_sample_age_us = 2000000U;
  jump_config.quality_timeout_us = 2000000U;
  VisionContract jump_contract(jump_config);
  makeHealthy(jump_contract, 1000000U);
  jump_contract.evaluate(sample(1000000U, 0.0, 0.0, 0.0), 1000000U);
  jump_contract.observeQuality(80, 1600000U);
  EXPECT_EQ(jump_contract.evaluate(sample(1600000U, 1.0, 0.0, 0.0), 1600000U).fault,
    FaultCode::kTimestampJump);
}

TEST(VisionContract, MissingTfAfterHealthyStartupGraceLatchesInputTimeout)
{
  ContractConfig config;
  config.maximum_sample_age_us = 200000U;
  config.quality_timeout_us = 1000000U;
  VisionContract contract(config);
  makeHealthy(contract, 1000000U);
  EXPECT_EQ(contract.tick(1000000U).decision, Decision::kWarmup);
  EXPECT_EQ(contract.tick(1200001U).fault, FaultCode::kInputTimeout);
  EXPECT_FALSE(contract.writerEnabled());
}

TEST(VisionContract, DiagnosticHistoryIsStrictlyBounded)
{
  vision_to_dds::BoundedHistory<int> history(3U);
  for (int value = 0; value < 10000; ++value) {
    history.push(value);
  }
  ASSERT_EQ(history.size(), 3U);
  EXPECT_EQ(history.values().front(), 9997);
  EXPECT_EQ(history.values().back(), 9999);
  history.setCapacity(0U);
  EXPECT_EQ(history.size(), 0U);
}

TEST(VisionContract, DuplicateWriterLatchesBeforeAnyOdometryCanPublish)
{
  VisionContract contract(ContractConfig{});
  makeHealthy(contract, 1000000U);
  contract.observeWriterCardinality(2U, 1000000U);
  EXPECT_EQ(contract.fault(), FaultCode::kDuplicateWriter);
  EXPECT_FALSE(contract.writerEnabled());
}

}  // namespace
