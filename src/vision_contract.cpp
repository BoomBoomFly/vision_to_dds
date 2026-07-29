#include "vision_to_dds/vision_contract.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace vision_to_dds
{
namespace
{
constexpr double kQuaternionNormTolerance = 0.01;

bool finiteVariance(const std::array<float, 3> & value)
{
  for (const float element : value) {
    if (!std::isfinite(element) || element < 0.0F) {
      return false;
    }
  }
  return true;
}
}  // namespace

const char * faultCodeName(FaultCode code)
{
  switch (code) {
    case FaultCode::kNone: return "NONE";
    case FaultCode::kConfiguration: return "CONFIGURATION";
    case FaultCode::kSourceEpochMissing: return "SOURCE_EPOCH_MISSING";
    case FaultCode::kSourceEpochChanged: return "SOURCE_EPOCH_CHANGED";
    case FaultCode::kQualityMissing: return "QUALITY_MISSING";
    case FaultCode::kQualityLow: return "QUALITY_LOW";
    case FaultCode::kQualityTimeout: return "QUALITY_TIMEOUT";
    case FaultCode::kFrameMismatch: return "FRAME_MISMATCH";
    case FaultCode::kInvalidSample: return "INVALID_SAMPLE";
    case FaultCode::kSampleTooOld: return "SAMPLE_TOO_OLD";
    case FaultCode::kSampleInFuture: return "SAMPLE_IN_FUTURE";
    case FaultCode::kTimestampRollback: return "TIMESTAMP_ROLLBACK";
    case FaultCode::kTimestampJump: return "TIMESTAMP_JUMP";
    case FaultCode::kClockRollback: return "CLOCK_ROLLBACK";
    case FaultCode::kInputTimeout: return "INPUT_TIMEOUT";
    case FaultCode::kDuplicateWriter: return "DUPLICATE_WRITER";
  }
  return "UNKNOWN";
}

VisionContract::VisionContract(ContractConfig config) : config_(std::move(config))
{
  if (!configurationValid()) {
    fault_ = FaultCode::kConfiguration;
  }
}

void VisionContract::observeSourceEpoch(uint32_t epoch, uint64_t now_us)
{
  if (have_last_now_ && now_us < last_now_us_) {
    latch(FaultCode::kClockRollback);
    return;
  }
  have_last_now_ = true;
  last_now_us_ = now_us;

  observed_epoch_ = epoch;
  if (!have_epoch_) {
    have_epoch_ = true;
    active_epoch_ = epoch;
  } else if (epoch != active_epoch_) {
    latch(FaultCode::kSourceEpochChanged);
  }
}

void VisionContract::observeQuality(int8_t quality, uint64_t now_us)
{
  if (have_last_now_ && now_us < last_now_us_) {
    latch(FaultCode::kClockRollback);
    return;
  }
  have_last_now_ = true;
  last_now_us_ = now_us;
  have_quality_ = true;
  quality_ = quality;
  quality_received_us_ = now_us;
  if (quality < config_.minimum_quality) {
    latch(FaultCode::kQualityLow);
  }
}

void VisionContract::observeWriterCardinality(std::size_t writer_count, uint64_t now_us)
{
  if (have_last_now_ && now_us < last_now_us_) {
    latch(FaultCode::kClockRollback);
    return;
  }
  have_last_now_ = true;
  last_now_us_ = now_us;
  if (writer_count != 1U) {
    latch(FaultCode::kDuplicateWriter);
  }
}

Evaluation VisionContract::tick(uint64_t now_us)
{
  if (have_last_now_ && now_us < last_now_us_) {
    return latch(FaultCode::kClockRollback);
  }
  have_last_now_ = true;
  last_now_us_ = now_us;
  if (fault_ != FaultCode::kNone) {
    return warmupOrFault();
  }
  // Before both side-channel health inputs arrive the bridge is simply
  // unarmed. It emits no PX4 data and does not turn normal startup ordering
  // into a fault that would require an unnecessary manual reset.
  if (!have_epoch_ || !have_quality_) {
    return warmupOrFault();
  }
  if (health_ready_since_us_ == 0U) {
    health_ready_since_us_ = now_us;
  }
  const FaultCode health = healthFault(now_us);
  if (health != FaultCode::kNone) {
    return latch(health);
  }
  if (have_sample_ && now_us > previous_sample_.timestamp_us + config_.maximum_sample_age_us) {
    return latch(FaultCode::kInputTimeout);
  }
  if (!have_sample_ && now_us > health_ready_since_us_ + config_.maximum_sample_age_us) {
    // Once measured health is available, a TF chain that never appears is an
    // input outage, not an indefinitely silent warm-up state.
    return latch(FaultCode::kInputTimeout);
  }
  return warmupOrFault();
}

Evaluation VisionContract::evaluate(const TransformSample & sample, uint64_t now_us)
{
  const Evaluation tick_result = tick(now_us);
  if (tick_result.decision == Decision::kFault) {
    return tick_result;
  }
  if (!have_epoch_ || !have_quality_) {
    return warmupOrFault();
  }
  if (sample.world_frame_id != config_.world_frame_id || sample.body_frame_id != config_.body_frame_id) {
    return latch(FaultCode::kFrameMismatch);
  }
  if (sample.timestamp_us == 0 || !finite(sample.position_enu) || !finite(sample.orientation_enu_flu)) {
    return latch(FaultCode::kInvalidSample);
  }
  const double norm = sample.orientation_enu_flu.length();
  if (!std::isfinite(norm) || std::fabs(norm - 1.0) > kQuaternionNormTolerance) {
    return latch(FaultCode::kInvalidSample);
  }
  if (sample.timestamp_us > now_us + config_.maximum_future_skew_us) {
    return latch(FaultCode::kSampleInFuture);
  }
  if (now_us > sample.timestamp_us + config_.maximum_sample_age_us) {
    return latch(FaultCode::kSampleTooOld);
  }
  if (!have_sample_) {
    previous_sample_ = sample;
    have_sample_ = true;
    return warmupOrFault();
  }
  if (sample.timestamp_us < previous_sample_.timestamp_us) {
    return latch(FaultCode::kTimestampRollback);
  }
  if (sample.timestamp_us == previous_sample_.timestamp_us) {
    return warmupOrFault();
  }
  const uint64_t delta_us = sample.timestamp_us - previous_sample_.timestamp_us;
  if (delta_us > config_.maximum_timestamp_jump_us) {
    return latch(FaultCode::kTimestampJump);
  }

  VehicleOdometrySample output;
  output.timestamp_us = sample.timestamp_us;
  output.timestamp_sample_us = sample.timestamp_us;
  output.position_ned = enuToNed(sample.position_enu);
  output.orientation_ned_frd = enuFluToNedFrd(sample.orientation_enu_flu);
  const tf2::Vector3 velocity_enu =
    (sample.position_enu - previous_sample_.position_enu) * (1000000.0 / static_cast<double>(delta_us));
  output.velocity_ned = enuToNed(velocity_enu);
  if (!finite(output.position_ned) || !finite(output.orientation_ned_frd) || !finite(output.velocity_ned)) {
    return latch(FaultCode::kInvalidSample);
  }
  output.position_variance_ned = enuToNedVariance(config_.position_variance_enu);
  output.orientation_variance_frd = enuToNedVariance(config_.orientation_variance_flu);
  output.velocity_variance_ned = enuToNedVariance(config_.velocity_variance_enu);
  output.reset_counter = reset_counter_;
  output.quality = quality_;
  previous_sample_ = sample;

  Evaluation result;
  result.decision = Decision::kPublish;
  result.odometry = output;
  return result;
}

void VisionContract::resetFault()
{
  fault_ = configurationValid() ? FaultCode::kNone : FaultCode::kConfiguration;
  have_sample_ = false;
  have_last_now_ = false;
  health_ready_since_us_ = 0U;
  if (have_epoch_) {
    active_epoch_ = observed_epoch_;
  }
  reset_counter_ = static_cast<uint8_t>(reset_counter_ + 1U);
}

Evaluation VisionContract::warmupOrFault()
{
  Evaluation result;
  if (fault_ != FaultCode::kNone) {
    result.decision = Decision::kFault;
    result.fault = fault_;
  }
  return result;
}

Evaluation VisionContract::latch(FaultCode code)
{
  if (fault_ == FaultCode::kNone) {
    fault_ = code;
  }
  return warmupOrFault();
}

bool VisionContract::configurationValid() const
{
  return !config_.world_frame_id.empty() && !config_.body_frame_id.empty() &&
    config_.maximum_sample_age_us > 0 && config_.quality_timeout_us > 0 &&
    config_.maximum_timestamp_jump_us > 0 && finiteVariance(config_.position_variance_enu) &&
    finiteVariance(config_.orientation_variance_flu) && finiteVariance(config_.velocity_variance_enu);
}

FaultCode VisionContract::healthFault(uint64_t now_us) const
{
  if (!have_epoch_) {
    return FaultCode::kSourceEpochMissing;
  }
  if (!have_quality_) {
    return FaultCode::kQualityMissing;
  }
  if (quality_ < config_.minimum_quality) {
    return FaultCode::kQualityLow;
  }
  if (now_us > quality_received_us_ + config_.quality_timeout_us) {
    return FaultCode::kQualityTimeout;
  }
  return FaultCode::kNone;
}

bool VisionContract::finite(const tf2::Vector3 & value)
{
  return std::isfinite(value.getX()) && std::isfinite(value.getY()) && std::isfinite(value.getZ());
}

bool VisionContract::finite(const tf2::Quaternion & value)
{
  return std::isfinite(value.getX()) && std::isfinite(value.getY()) &&
    std::isfinite(value.getZ()) && std::isfinite(value.getW());
}

tf2::Vector3 VisionContract::enuToNed(const tf2::Vector3 & value)
{
  return tf2::Vector3(value.getY(), value.getX(), -value.getZ());
}

tf2::Quaternion VisionContract::enuFluToNedFrd(const tf2::Quaternion & value)
{
  const double root_half = std::sqrt(0.5);
  const tf2::Quaternion ned_from_enu(root_half, root_half, 0.0, 0.0);
  const tf2::Quaternion flu_from_frd(1.0, 0.0, 0.0, 0.0);
  tf2::Quaternion result = ned_from_enu * value * flu_from_frd;
  result.normalize();
  return result;
}

std::array<float, 3> VisionContract::enuToNedVariance(const std::array<float, 3> & value)
{
  return {{value[1], value[0], value[2]}};
}

}  // namespace vision_to_dds
