#ifndef VISION_TO_DDS__VISION_CONTRACT_HPP_
#define VISION_TO_DDS__VISION_CONTRACT_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace vision_to_dds
{

// These reasons are deliberately stable: permanent contract faults emit their
// names in a structured fault event and require resetFault(). QUALITY_LOW is
// different: it is a temporary warm-up inhibition and clears automatically
// once healthy quality and fresh TF samples arrive.
enum class FaultCode {
  kNone,
  kConfiguration,
  kSourceEpochMissing,
  kSourceEpochChanged,
  kQualityMissing,
  kQualityLow,
  kQualityTimeout,
  kFrameMismatch,
  kInvalidSample,
  kSampleTooOld,
  kSampleInFuture,
  kTimestampRollback,
  kTimestampJump,
  kClockRollback,
  kInputTimeout,
  kDuplicateWriter,
};

const char * faultCodeName(FaultCode code);

enum class Decision { kWarmup, kPublish, kFault };

struct ContractConfig {
  // The vehicle contract is the body pose, not the T265 sensor pose.  The
  // measured t265_pose_frame -> base_link extrinsic is supplied at deployment.
  std::string world_frame_id{"odom_frame"};
  std::string body_frame_id{"base_link"};
  int8_t minimum_quality{50};
  uint64_t maximum_sample_age_us{200000};
  uint64_t maximum_future_skew_us{20000};
  uint64_t maximum_timestamp_jump_us{500000};
  uint64_t quality_timeout_us{250000};
  std::array<float, 3> position_variance_enu{{0.04F, 0.04F, 0.09F}};
  std::array<float, 3> orientation_variance_flu{{0.04F, 0.04F, 0.09F}};
  std::array<float, 3> velocity_variance_enu{{0.25F, 0.25F, 0.36F}};
};

// Transform stamped as world(ENU) <- body(FLU).  Timestamps must be from the
// same synchronized ROS/XRCE time base that PX4 uses for external vision.
struct TransformSample {
  std::string world_frame_id;
  std::string body_frame_id;
  uint64_t timestamp_us{0};
  tf2::Vector3 position_enu;
  tf2::Quaternion orientation_enu_flu;
};

struct VehicleOdometrySample {
  uint64_t timestamp_us{0};
  uint64_t timestamp_sample_us{0};
  tf2::Vector3 position_ned;
  tf2::Quaternion orientation_ned_frd;
  tf2::Vector3 velocity_ned;
  std::array<float, 3> position_variance_ned{};
  std::array<float, 3> orientation_variance_frd{};
  std::array<float, 3> velocity_variance_ned{};
  uint8_t reset_counter{0};
  int8_t quality{0};
};

struct Evaluation {
  Decision decision{Decision::kWarmup};
  FaultCode fault{FaultCode::kNone};
  VehicleOdometrySample odometry{};
};

// Used for optional diagnostics only. Retaining debug history must never turn
// a healthy flight into an unbounded-memory process.
template<typename T>
class BoundedHistory
{
public:
  explicit BoundedHistory(std::size_t capacity) : capacity_(capacity) {}

  void setCapacity(std::size_t capacity)
  {
    capacity_ = capacity;
    trim();
  }

  void push(const T & value)
  {
    if (capacity_ == 0U) {
      return;
    }
    values_.push_back(value);
    trim();
  }

  std::size_t size() const { return values_.size(); }
  const std::deque<T> & values() const { return values_; }

private:
  void trim()
  {
    while (values_.size() > capacity_) {
      values_.pop_front();
    }
  }

  std::size_t capacity_;
  std::deque<T> values_;
};

// A fail-closed state machine. Health, source identity and timestamps are
// inputs, not values invented by this bridge. Permanent contract faults
// require an explicit operator reset after the upstream source is healthy
// again; transient low tracking quality remains in warm-up and self-recovers.
class VisionContract
{
public:
  explicit VisionContract(ContractConfig config);

  void observeSourceEpoch(uint32_t epoch, uint64_t now_us);
  void observeQuality(int8_t quality, uint64_t now_us);
  void observeWriterCardinality(std::size_t writer_count, uint64_t now_us);
  Evaluation tick(uint64_t now_us);
  Evaluation evaluate(const TransformSample & sample, uint64_t now_us);
  void resetFault();

  FaultCode fault() const { return fault_; }
  uint8_t resetCounter() const { return reset_counter_; }
  bool writerEnabled() const { return fault_ == FaultCode::kNone; }

private:
  Evaluation warmupOrFault();
  Evaluation latch(FaultCode code);
  bool configurationValid() const;
  FaultCode healthFault(uint64_t now_us) const;
  static bool finite(const tf2::Vector3 & value);
  static bool finite(const tf2::Quaternion & value);
  static tf2::Vector3 enuToNed(const tf2::Vector3 & value);
  static tf2::Quaternion enuFluToNedFrd(const tf2::Quaternion & value);
  static std::array<float, 3> enuToNedVariance(const std::array<float, 3> & value);

  ContractConfig config_;
  FaultCode fault_{FaultCode::kNone};
  bool have_epoch_{false};
  uint32_t observed_epoch_{0};
  uint32_t active_epoch_{0};
  bool have_quality_{false};
  int8_t quality_{0};
  uint64_t quality_received_us_{0};
  uint64_t health_ready_since_us_{0};
  bool have_last_now_{false};
  uint64_t last_now_us_{0};
  bool have_sample_{false};
  TransformSample previous_sample_{};
  uint8_t reset_counter_{0};
};

}  // namespace vision_to_dds

#endif  // VISION_TO_DDS__VISION_CONTRACT_HPP_
