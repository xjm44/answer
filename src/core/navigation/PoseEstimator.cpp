#include "core/navigation/PoseEstimator.h"

#include <algorithm>
#include <cmath>

#include "core/common/Geometry.h"
#include "core/common/Log.h"

namespace vacuum {

void PoseEstimator::reset() {
  VLOG_I(lcPose) << "reset: pose unknown, initialising from the next measurements";
  valid_ = false;
  samples_ = 0;
  outliersInRow_ = 0;
  haveTicks_ = false;
}

void PoseEstimator::reset(const Pose2D& exact) {
  VLOG_I(lcPose) << "reset to exact pose " << exact;
  pose_ = exact;
  valid_ = true;
  samples_ = config_.initialSamples;  // trust it like a fully averaged estimate
  outliersInRow_ = 0;
  haveTicks_ = false;
}

const Pose2D& PoseEstimator::update(const RobotState& state) {
  applyOdometry(state);
  if (!valid_) {
    pose_ = state.pose;
    valid_ = true;
    samples_ = 1;
    outliersInRow_ = 0;
    VLOG_I(lcPose) << "initialised from the first measurement " << pose_;
    return pose_;
  }
  applyMeasurement(state.pose);
  return pose_;
}

void PoseEstimator::applyOdometry(const RobotState& state) {
  if (!haveTicks_) {
    lastLeftTicks_ = state.leftTicks;
    lastRightTicks_ = state.rightTicks;
    haveTicks_ = true;
    return;
  }
  const std::int64_t dLeft = state.leftTicks - lastLeftTicks_;
  const std::int64_t dRight = state.rightTicks - lastRightTicks_;
  lastLeftTicks_ = state.leftTicks;
  lastRightTicks_ = state.rightTicks;

  // More than a wheel revolution between two messages is impossible at 10 rad/s:
  // the counters were reset by load/reset. Re-baseline without moving.
  if (std::llabs(dLeft) > info_.ticksPerRev || std::llabs(dRight) > info_.ticksPerRev) {
    VLOG_I(lcPose) << "encoder counters jumped (dL " << dLeft << ", dR " << dRight << "): load/reset, re-baselined";
    return;
  }
  if (!valid_) return;

  const double metresPerTick = 2.0 * kPi * info_.wheelRadius / info_.ticksPerRev;
  const double left = static_cast<double>(dLeft) * metresPerTick;
  const double right = static_cast<double>(dRight) * metresPerTick;
  const double ds = 0.5 * (left + right);
  const double dYaw = (right - left) / info_.wheelBase;
  pose_.x += ds * std::cos(pose_.yaw + 0.5 * dYaw);
  pose_.y += ds * std::sin(pose_.yaw + 0.5 * dYaw);
  pose_.yaw = wrapAngle(pose_.yaw + dYaw);
}

void PoseEstimator::applyMeasurement(const Pose2D& measured) {
  const double dx = measured.x - pose_.x;
  const double dy = measured.y - pose_.y;
  const double dYaw = wrapAngle(measured.yaw - pose_.yaw);

  if (std::hypot(dx, dy) > config_.gateDistance || std::abs(dYaw) > config_.gateYaw) {
    if (outliersInRow_ == 0)
      VLOG_D(lcPose) << "outlier ignored: measured " << measured << " vs estimate " << pose_ << " ("
                     << QString::number(std::hypot(dx, dy) * 100.0, 'f', 1) << " cm, "
                     << QString::number(radToDeg(dYaw), 'f', 1) << "°)";
    if (++outliersInRow_ >= config_.resyncAfter) {  // not a glitch: we are lost, start over
      VLOG_W(lcPose) << outliersInRow_ << " outliers in a row: re-initialising at the measured pose " << measured
                     << " (estimate was " << pose_ << ")";
      pose_ = measured;
      samples_ = 1;
      outliersInRow_ = 0;
    }
    return;
  }
  if (outliersInRow_ > 0) VLOG_D(lcPose) << "outlier run ended after " << outliersInRow_ << " measurement(s)";
  outliersInRow_ = 0;

  // Running average while initialising, then a constant small gain.
  ++samples_;
  const double gain = std::max(config_.correctionGain, 1.0 / samples_);
  pose_.x += gain * dx;
  pose_.y += gain * dy;
  pose_.yaw = wrapAngle(pose_.yaw + gain * dYaw);
}

}  // namespace vacuum
