#ifndef VISION_TO_DDS__T265_HEALTH_ADAPTER_HPP_
#define VISION_TO_DDS__T265_HEALTH_ADAPTER_HPP_

#include <array>
#include <cstdint>

namespace vision_to_dds
{

// This is the health-only boundary for the legacy RealSense ROS T265 driver.
// That driver exports tracker_confidence through its Odometry covariance:
// linear_accel_cov * 10^(3 - tracker_confidence).  We deliberately consume
// that measured covariance instead of inventing a bridge-local quality value.
struct T265HealthConfig {
  uint64_t stream_timeout_us{200000};
  double linear_accel_covariance{0.01};
};

struct T265OdometryHealthSample {
  uint64_t timestamp_us{0};
  std::array<double, 36> pose_covariance{};
};

struct T265HealthOutput {
  uint32_t source_epoch{1U};
  int8_t quality{0};
  // False until the adapter has actually observed a T265 odometry message.
  // This prevents normal process startup from fabricating a measured
  // quality=0 sample and latching the downstream bridge before the camera's
  // first message can arrive.
  bool quality_available{false};
  bool source_epoch_changed{false};
};

// Converts real T265 driver health evidence to the two upstream inputs of the
// visual-odometry writer contract.  It never creates TF or alters a TF stamp.
// A stale stream, repeated/rollback source stamp, invalid covariance, or
// local clock rollback publishes quality 0.  A stream that resumes after a
// timeout obtains a new source epoch, forcing the downstream bridge through
// its latched-fault/reset/warm-up path.
class T265HealthAdapter
{
public:
  explicit T265HealthAdapter(T265HealthConfig config);

  T265HealthOutput observeOdometry(const T265OdometryHealthSample & sample, uint64_t now_us);
  T265HealthOutput tick(uint64_t now_us);

  uint32_t sourceEpoch() const { return source_epoch_; }
  int8_t quality() const { return quality_; }

private:
  bool configurationValid() const;
  static bool finiteCovariance(const std::array<double, 36> & covariance);
  int8_t measuredQuality(const std::array<double, 36> & covariance) const;
  T265HealthOutput output(bool source_epoch_changed) const;
  void startNewEpoch();

  T265HealthConfig config_;
  uint32_t source_epoch_{1U};
  int8_t quality_{0};
  bool have_last_now_{false};
  uint64_t last_now_us_{0};
  bool have_sample_{false};
  uint64_t last_sample_timestamp_us_{0};
  uint64_t last_received_us_{0};
  bool have_observed_source_{false};
  bool timed_out_{false};
};

}  // namespace vision_to_dds

#endif  // VISION_TO_DDS__T265_HEALTH_ADAPTER_HPP_
