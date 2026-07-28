#include "vision_to_dds/t265_health_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace vision_to_dds
{

T265HealthAdapter::T265HealthAdapter(T265HealthConfig config)
: config_(std::move(config))
{
  if (!configurationValid()) {
    quality_ = 0;
  }
}

T265HealthOutput T265HealthAdapter::observeOdometry(
  const T265OdometryHealthSample & sample, uint64_t now_us)
{
  bool new_epoch = false;
  if (!configurationValid() || (have_last_now_ && now_us < last_now_us_)) {
    startNewEpoch();
    quality_ = 0;
    have_sample_ = false;
    timed_out_ = true;
    last_now_us_ = now_us;
    have_last_now_ = true;
    return output(true);
  }
  have_last_now_ = true;
  last_now_us_ = now_us;

  // A timeout means the device/driver session was absent.  Its returning
  // output is a new source epoch even if the reconstructed ROS stamp happens
  // to continue increasing across the reconnect.
  if (timed_out_) {
    startNewEpoch();
    new_epoch = true;
    timed_out_ = false;
    have_sample_ = false;
  }

  // Do not reinterpret a clock reset or a frozen/replayed source stamp as a
  // valid tracking sample.  A strict regression is a source restart; an exact
  // repeat is a freeze and keeps the same epoch so it cannot mask a fault.
  if (sample.timestamp_us == 0U || !finiteCovariance(sample.pose_covariance)) {
    quality_ = 0;
    return output(new_epoch);
  }
  if (have_sample_ && sample.timestamp_us < last_sample_timestamp_us_) {
    startNewEpoch();
    quality_ = 0;
    have_sample_ = false;
    timed_out_ = true;
    return output(true);
  }
  if (have_sample_ && sample.timestamp_us == last_sample_timestamp_us_) {
    quality_ = 0;
    return output(new_epoch);
  }

  have_sample_ = true;
  last_sample_timestamp_us_ = sample.timestamp_us;
  last_received_us_ = now_us;
  quality_ = measuredQuality(sample.pose_covariance);
  return output(new_epoch);
}

T265HealthOutput T265HealthAdapter::tick(uint64_t now_us)
{
  if (!configurationValid() || (have_last_now_ && now_us < last_now_us_)) {
    quality_ = 0;
    timed_out_ = true;
    have_sample_ = false;
    last_now_us_ = now_us;
    have_last_now_ = true;
    return output(false);
  }
  have_last_now_ = true;
  last_now_us_ = now_us;
  if (!have_sample_ || now_us > last_received_us_ + config_.stream_timeout_us) {
    quality_ = 0;
    timed_out_ = true;
  }
  return output(false);
}

bool T265HealthAdapter::configurationValid() const
{
  return config_.stream_timeout_us > 0U && std::isfinite(config_.linear_accel_covariance) &&
    config_.linear_accel_covariance > 0.0;
}

bool T265HealthAdapter::finiteCovariance(const std::array<double, 36> & covariance)
{
  // The legacy driver fills all translational pose diagonals from the same
  // tracker confidence.  Require those values to be finite and non-negative.
  constexpr std::array<std::size_t, 3> kPositionDiagonal{{0U, 7U, 14U}};
  for (const std::size_t index : kPositionDiagonal) {
    if (!std::isfinite(covariance[index]) || covariance[index] <= 0.0) {
      return false;
    }
  }
  return true;
}

int8_t T265HealthAdapter::measuredQuality(const std::array<double, 36> & covariance) const
{
  const double variance = (covariance[0] + covariance[7] + covariance[14]) / 3.0;
  if (!std::isfinite(variance) || variance <= 0.0) {
    return 0;
  }
  // This reverses the legacy driver's documented covariance encoding.  The
  // resulting values are intentionally discrete, measured confidence levels:
  // tracker confidence 0, 1, 2, 3 -> quality 0, 33, 67, 100.
  const double encoded = std::log10(std::max(variance, std::numeric_limits<double>::min()) /
      config_.linear_accel_covariance);
  const int confidence = std::max(0, std::min(3, 3 - static_cast<int>(std::lround(encoded))));
  return static_cast<int8_t>((100 * confidence) / 3);
}

T265HealthOutput T265HealthAdapter::output(bool source_epoch_changed) const
{
  T265HealthOutput result;
  result.source_epoch = source_epoch_;
  result.quality = quality_;
  result.source_epoch_changed = source_epoch_changed;
  return result;
}

void T265HealthAdapter::startNewEpoch()
{
  source_epoch_ = static_cast<uint32_t>(source_epoch_ + 1U);
  if (source_epoch_ == 0U) {
    source_epoch_ = 1U;
  }
}

}  // namespace vision_to_dds
