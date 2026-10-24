#pragma once

#include <cstdint>

#include "core/common/Types.h"

namespace vacuum {

// Fuses wheel odometry with the noisy pose of the state stream.
//
// The streamed pose jitters by about 1 cm / 0.5 deg (std) around the truth, which is
// as large as the 5 cm goal tolerance. Encoder ticks are smooth but drift. This is a
// complementary filter: odometry predicts the motion between messages, and every
// measurement pulls the estimate a small step towards itself, which averages the noise
// without lagging behind the motion.
//
// It is self-healing: tick counter resets (load/reset) are detected, outlier measurements
// are ignored, and a persistent disagreement (robot moved by someone else) re-initialises.
class PoseEstimator {
 public:
  struct Config {
    double correctionGain = 0.04;  // per measurement at 50 Hz: ~0.5 s time constant
    int initialSamples = 25;       // average this many measurements when the pose is unknown
    double gateDistance = 0.30;    // measurements further off than this are treated as outliers
    double gateYaw = 0.6;          // rad
    int resyncAfter = 50;          // consecutive outliers before re-initialising (~1 s)
  };

  PoseEstimator() = default;
  explicit PoseEstimator(const Config& config) : config_(config) {}

  void setRobotInfo(const RobotInfo& info) { info_ = info; }

  void reset();                     // pose unknown: initialise from the next measurements
  void reset(const Pose2D& exact);  // pose known exactly, e.g. the start pose of load_scene

  // Feeds one state message; returns the new estimate.
  const Pose2D& update(const RobotState& state);

  bool isValid() const { return valid_; }
  const Pose2D& pose() const { return pose_; }

 private:
  void applyOdometry(const RobotState& state);
  void applyMeasurement(const Pose2D& measured);

  Config config_;
  RobotInfo info_;

  Pose2D pose_;
  bool valid_ = false;
  int samples_ = 0;
  int outliersInRow_ = 0;

  bool haveTicks_ = false;
  std::int64_t lastLeftTicks_ = 0;
  std::int64_t lastRightTicks_ = 0;
};

}  // namespace vacuum
