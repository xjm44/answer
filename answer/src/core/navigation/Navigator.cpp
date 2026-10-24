#include "core/navigation/Navigator.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/navigation/PathPlanner.h"
#include "core/sim/Protocol.h"
#include "core/sim/SimClient.h"

namespace vacuum {

namespace {

double clampAbs(double value, double limit) { return std::clamp(value, -limit, limit); }

// Proportional turn command with a floor so small errors still get corrected.
double turnCommand(double error, double gain, double minRate, double maxRate) {
  const double rate = std::clamp(std::abs(gain * error), minRate, maxRate);
  return error >= 0.0 ? rate : -rate;
}

QString formatTarget(const NavTarget& t) {
  if (!t.goalName.empty()) return QString::fromStdString(t.goalName);
  return QStringLiteral("(%1, %2)").arg(t.x, 0, 'f', 2).arg(t.y, 0, 'f', 2);
}

}  // namespace

const char* Navigator::toString(Mode mode) {
  switch (mode) {
    case Mode::Idle: return "Idle";
    case Mode::Manual: return "Manual";
    case Mode::Following: return "Following path";
    case Mode::FinalApproach: return "Final approach";
    case Mode::Aligning: return "Aligning";
    case Mode::Settling: return "Settling";
    case Mode::Recovering: return "Recovering";
    case Mode::Arrived: return "Arrived";
    case Mode::Failed: return "Failed";
  }
  return "?";
}

Navigator::Navigator(QObject* parent) : QObject(parent) {
  timer_.setTimerType(Qt::PreciseTimer);
  timer_.setInterval(config_.tickMs);
  connect(&timer_, &QTimer::timeout, this, &Navigator::tick);
  timer_.start();
}

void Navigator::setClient(SimClient* client) {
  if (client_ && client != client_) stop();
  client_ = client;
  haveState_ = false;
}

void Navigator::setMap(const OccupancyMap& map) {
  stop();
  CostMapConfig costConfig;
  costConfig.robotRadius = info_.radius;
  VLOG_I(lcNav) << "new map: rebuilding the cost map (learned obstacles are forgotten)";
  costMap_.build(map, costConfig);
  emit costMapChanged();
}

bool Navigator::isBusy() const {
  return mode_ != Mode::Idle && mode_ != Mode::Arrived && mode_ != Mode::Failed;
}

void Navigator::onState(const RobotState& state, const Pose2D& estimate) {
  pose_ = estimate;
  bumper_ = state.bumper;
  haveState_ = true;
  stateAge_.restart();

  // React at once (not on the next tick); Recovering is excluded, so this fires once per hit.
  const bool drivingItself = mode_ == Mode::Following || mode_ == Mode::FinalApproach || mode_ == Mode::Aligning;
  if (bumper_ && drivingItself) {
    VLOG_W(lcNav) << "bumper pressed while " << toString(mode_) << " at " << pose_ << ", last command v "
                  << QString::number(lastLinear_, 'f', 3) << " m/s, w " << QString::number(lastAngular_, 'f', 3)
                  << " rad/s";
    startRecovery();
  }
}

// ---------------------------------------------------------------------------
// Commands from the UI
// ---------------------------------------------------------------------------

bool Navigator::navigateTo(const NavTarget& target) {
  VLOG_I(lcNav) << "navigate to " << formatTarget(target) << " (" << QString::number(target.x, 'f', 3) << ", "
                << QString::number(target.y, 'f', 3) << ")"
                << (target.yaw ? QStringLiteral(" yaw %1°").arg(radToDeg(*target.yaw), 0, 'f', 1)
                               : QStringLiteral(" any heading"))
                << " from " << pose_ << ", bumper " << (bumper_ ? "PRESSED" : "clear") << ", previous mode "
                << toString(mode_) << ", " << costMap_.learnedObstacles().size() << " learned obstacle(s)";
  if (!client_) {
    fail(tr("not connected to the simulator"));
    return false;
  }
  if (!costMap_.isValid()) {
    fail(tr("no map loaded"));
    return false;
  }
  if (!haveState_ || stateAge_.elapsed() > config_.stateTimeoutMs) {
    fail(haveState_ ? tr("no robot state for %1 ms").arg(stateAge_.elapsed()) : tr("no robot state received"));
    return false;
  }
  target_ = target;
  collisions_ = 0;
  collisionSpots_.clear();
  replans_ = 0;
  approachAttempts_ = 0;
  if (!planPath()) return false;
  setMode(Mode::Following, tr("driving to %1").arg(formatTarget(target)));
  return true;
}

void Navigator::driveManual(double linear, double angular) {
  if (linear == 0.0 && angular == 0.0) {
    if (mode_ == Mode::Manual) {
      VLOG_I(lcNav) << "manual drive released at " << pose_;
      sendWheels(0.0, 0.0);
      stopsPending_ = 3;
      setMode(Mode::Idle, tr("manual drive released"));
    }
    return;
  }
  if (mode_ != Mode::Manual) {
    if (isBusy()) VLOG_I(lcNav) << "manual drive takes over from " << toString(mode_);
    target_.reset();
    clearPath();
    setMode(Mode::Manual, tr("driving by hand"));
  }
  VLOG_I(lcNav) << "manual drive v " << QString::number(linear, 'f', 3) << " m/s, w "
                << QString::number(angular, 'f', 3) << " rad/s from " << pose_;
  manualLinear_ = linear;
  manualAngular_ = angular;
  sendWheels(linear, angular);
}

void Navigator::stop() {
  const bool wasBusy = isBusy();
  if (wasBusy) VLOG_I(lcNav) << "stop requested while " << toString(mode_) << " at " << pose_;
  target_.reset();
  clearPath();
  sendWheels(0.0, 0.0);
  stopsPending_ = 3;
  if (wasBusy) setMode(Mode::Idle, tr("stopped"));
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

void Navigator::tick() {
  if (!isBusy()) {
    if (stopsPending_ > 0) {
      --stopsPending_;
      sendWheels(0.0, 0.0);
    }
    return;
  }
  // Never drive blind: without fresh state, hold still until it comes back.
  if (!haveState_ || stateAge_.elapsed() > config_.stateTimeoutMs) {
    if (staleLog_.ready())
      VLOG_W(lcNav) << "holding still: no robot state for " << (haveState_ ? stateAge_.elapsed() : -1) << " ms";
    sendWheels(0.0, 0.0);
    return;
  }
  switch (mode_) {
    case Mode::Manual: sendWheels(manualLinear_, manualAngular_); break;
    case Mode::Following: followPath(); break;
    case Mode::FinalApproach: finalApproach(); break;
    case Mode::Aligning: align(); break;
    case Mode::Settling: settle(); break;
    case Mode::Recovering: recover(); break;
    default: break;
  }
}

void Navigator::followPath() {
  if (path_.size() < 2) {
    setMode(Mode::FinalApproach);
    finalApproach();
    return;
  }
  const Point2D p{pose_.x, pose_.y};

  // Progress along the path: closest point on the current and next few segments.
  double bestDistance = std::numeric_limits<double>::infinity();
  Point2D bestPoint = path_[segment_];
  std::size_t bestSegment = segment_;
  const std::size_t lastSegment = std::min(path_.size() - 1, segment_ + 4);
  for (std::size_t i = segment_; i < lastSegment; ++i) {
    const Point2D q = projectOntoSegment(p, path_[i], path_[i + 1]);
    const double d = distance(p, q);
    if (d < bestDistance) {
      bestDistance = d;
      bestPoint = q;
      bestSegment = i;
    }
  }
  if (bestSegment != segment_)
    VLOG_D(lcNav) << "path segment " << segment_ << " -> " << bestSegment << " of " << path_.size() - 1 << " at "
                  << pose_;
  segment_ = bestSegment;
  if (bestDistance > config_.offPathDistance) {
    VLOG_W(lcNav) << "off the path by " << QString::number(bestDistance, 'f', 3) << " m (limit "
                  << config_.offPathDistance << ") at " << pose_;
    replan(tr("left the path"));
    return;
  }

  double remaining = distance(bestPoint, path_[segment_ + 1]);
  for (std::size_t i = segment_ + 1; i + 1 < path_.size(); ++i) remaining += distance(path_[i], path_[i + 1]);
  if (remaining < config_.finalApproachDistance) {
    VLOG_I(lcNav) << "path end " << QString::number(remaining, 'f', 3) << " m away: final approach from " << pose_;
    approachDriving_ = false;
    setMode(Mode::FinalApproach, tr("approaching %1").arg(formatTarget(*target_)));
    finalApproach();
    return;
  }

  // Tightest spot at the robot and on the path just ahead.
  double clearance = clearanceAt(pose_);
  for (double s = 0.02; s <= config_.tightPreview; s += 0.02)
    clearance = std::min(clearance, costMap_.clearanceAt(pointAlongPath(segment_, bestPoint, s)));
  const double tight = tightness(clearance);

  double lookahead =
      std::clamp(config_.lookaheadMin + 0.6 * std::abs(lastLinear_), config_.lookaheadMin, config_.lookaheadMax);
  lookahead += tight * (config_.tightLookahead - lookahead);
  const Point2D aim = pointAlongPath(segment_, bestPoint, lookahead);
  const double alpha = wrapAngle(std::atan2(aim.y - p.y, aim.x - p.x) - pose_.yaw);

  // Large heading error (start, sharp corner): turn on the spot first. Hysteresis avoids chattering.
  const double turnThreshold = turningInPlace_ ? config_.turnInPlaceExit : config_.turnInPlaceEnter;
  if (std::abs(alpha) > turnThreshold) {
    if (!turningInPlace_)
      VLOG_D(lcNav) << "heading off by " << QString::number(radToDeg(alpha), 'f', 1) << "°: turning in place";
    turningInPlace_ = true;
    sendWheels(0.0, turnCommand(alpha, config_.turnGain, config_.minTurnRate, config_.maxAngularSpeed));
    if (tickLog_.ready())
      VLOG_D(lcNavTick) << "follow: turn in place, pose " << pose_ << " aim " << aim << " alpha "
                        << QString::number(radToDeg(alpha), 'f', 1) << "° w " << QString::number(lastAngular_, 'f', 2);
    return;
  }
  if (turningInPlace_) VLOG_D(lcNav) << "heading within " << QString::number(radToDeg(alpha), 'f', 1) << "°: driving";
  turningInPlace_ = false;

  // Speed: cruise, slow down towards the end so we enter the final approach gently,
  // and slow down when the heading is off.
  const double brakingRoom = std::max(0.0, remaining - config_.finalApproachDistance);
  double v = std::min(config_.cruiseSpeed, config_.approachSpeed + std::sqrt(2.0 * config_.deceleration * brakingRoom));
  v = std::min(v, config_.cruiseSpeed + tight * (config_.tightSpeed - config_.cruiseSpeed));
  v *= std::max(0.3, std::cos(alpha));
  // Pure pursuit: arc through the look-ahead point.
  const double curvature = 2.0 * std::sin(alpha) / lookahead;
  double w = v * curvature;
  if (std::abs(w) > config_.maxAngularSpeed) {
    v *= config_.maxAngularSpeed / std::abs(w);
    w = clampAbs(w, config_.maxAngularSpeed);
  }
  sendWheels(v, w);
  if (tickLog_.ready())
    VLOG_D(lcNavTick) << "follow: pose " << pose_ << " seg " << segment_ << "/" << path_.size() - 1 << " off-path "
                      << QString::number(bestDistance, 'f', 3) << " remaining " << QString::number(remaining, 'f', 2)
                      << " lookahead " << QString::number(lookahead, 'f', 2) << " aim " << aim << " alpha "
                      << QString::number(radToDeg(alpha), 'f', 1) << "° -> v " << QString::number(v, 'f', 3) << " w "
                      << QString::number(w, 'f', 3) << " clearance here "
                      << QString::number(clearanceAt(pose_), 'f', 3) << " min ahead " << QString::number(clearance, 'f', 3)
                      << " tight " << QString::number(tight, 'f', 2);
}

// Point approach: turn to face the goal (or face away and back up if it is just behind),
// drive straight, stop when level with it. Repeats if a sideways error remains.
void Navigator::finalApproach() {
  const double dx = target_->x - pose_.x;
  const double dy = target_->y - pose_.y;
  const double dist = std::hypot(dx, dy);
  if (dist <= config_.positionTolerance) {
    VLOG_I(lcNav) << "position reached: " << QString::number(dist * 100.0, 'f', 1) << " cm off at " << pose_;
    sendWheels(0.0, 0.0);
    approachDriving_ = false;
    afterPositionReached();
    return;
  }
  if (dist > 3.0 * config_.finalApproachDistance) {
    VLOG_W(lcNav) << "final approach: " << QString::number(dist, 'f', 3) << " m from the goal, too far";
    replan(tr("drifted away from the goal"));
    return;
  }

  const double bearing = std::atan2(dy, dx);
  if (!approachDriving_) {
    approachReverse_ = dist < config_.reverseDistance && std::abs(wrapAngle(bearing - pose_.yaw)) > kPi / 2;
    const double facing = approachReverse_ ? wrapAngle(bearing + kPi) : bearing;
    const double alpha = wrapAngle(facing - pose_.yaw);
    if (std::abs(alpha) > config_.approachAlignTolerance) {
      sendWheels(0.0, turnCommand(alpha, config_.turnGain, config_.minTurnRate, config_.alignMaxRate));
      if (tickLog_.ready())
        VLOG_D(lcNavTick) << "approach: facing the goal" << (approachReverse_ ? " (backwards)" : "") << ", pose "
                          << pose_ << " dist " << QString::number(dist, 'f', 3) << " alpha "
                          << QString::number(radToDeg(alpha), 'f', 1) << "°";
      return;
    }
    approachDriving_ = true;
    VLOG_D(lcNav) << "approach attempt " << approachAttempts_ + 1 << ": driving " << (approachReverse_ ? "backwards" : "forwards")
                  << " " << QString::number(dist * 100.0, 'f', 1) << " cm to the goal from " << pose_;
  }

  const double direction = approachReverse_ ? -1.0 : 1.0;
  const double along = direction * dist * std::cos(bearing - pose_.yaw);  // remaining distance in travel direction
  if (along <= 0.003) {  // level with the goal (or past it): stop and re-evaluate
    sendWheels(0.0, 0.0);
    approachDriving_ = false;
    VLOG_D(lcNav) << "approach: level with the goal, " << QString::number(dist * 100.0, 'f', 1) << " cm off at "
                  << pose_;
    if (++approachAttempts_ > config_.maxApproachAttempts) {
      if (dist <= config_.markPositionTolerance) afterPositionReached();
      else fail(tr("could not settle on the goal position (%1 cm off)").arg(dist * 100.0, 0, 'f', 1));
    }
    return;
  }
  const double facing = approachReverse_ ? wrapAngle(bearing + kPi) : bearing;
  const double alpha = wrapAngle(facing - pose_.yaw);
  const double v = std::min(config_.approachSpeed, config_.approachGain * along + 0.01);
  const double w = dist > 0.04 ? clampAbs(2.0 * alpha, 0.5) : 0.0;  // heading is unreliable very close
  sendWheels(direction * v, w);
  if (tickLog_.ready())
    VLOG_D(lcNavTick) << "approach: pose " << pose_ << " dist " << QString::number(dist, 'f', 3) << " along "
                      << QString::number(along, 'f', 3) << " -> v " << QString::number(direction * v, 'f', 3) << " w "
                      << QString::number(w, 'f', 3);
}

void Navigator::afterPositionReached() {
  if (target_->yaw) setMode(Mode::Aligning, tr("turning to %1°").arg(*target_->yaw * 180.0 / kPi, 0, 'f', 0));
  else beginSettling();
}

void Navigator::align() {
  const double error = yawError();
  if (std::abs(error) <= config_.yawTolerance) {
    VLOG_I(lcNav) << "heading reached: " << QString::number(radToDeg(error), 'f', 2) << "° off at " << pose_;
    sendWheels(0.0, 0.0);
    beginSettling();
    return;
  }
  sendWheels(0.0, turnCommand(error, 3.0, config_.minTurnRate, config_.alignMaxRate));
  if (tickLog_.ready())
    VLOG_D(lcNavTick) << "align: yaw " << QString::number(radToDeg(pose_.yaw), 'f', 1) << "° error "
                      << QString::number(radToDeg(error), 'f', 1) << "° w " << QString::number(lastAngular_, 'f', 2);
}

void Navigator::beginSettling() {
  sendWheels(0.0, 0.0);
  settleTimer_.restart();
  setMode(Mode::Settling, tr("checking the final pose"));
}

// Stand still so the pose estimate converges, then verify before marking:
// only the first mark_goal per goal counts, so it must not be wasted.
void Navigator::settle() {
  sendWheels(0.0, 0.0);
  if (settleTimer_.elapsed() < config_.settleMs) return;

  const bool positionOk = positionError() <= config_.markPositionTolerance;
  const bool yawOk = std::abs(yawError()) <= config_.markYawTolerance;
  VLOG_I(lcNav) << "settled at " << pose_ << ": " << QString::number(positionError() * 100.0, 'f', 1) << " cm "
                << (positionOk ? "ok" : "TOO FAR") << ", " << QString::number(radToDeg(std::abs(yawError())), 'f', 1)
                << "° " << (yawOk ? "ok" : "TOO FAR") << " (limits " << config_.markPositionTolerance * 100.0
                << " cm, " << radToDeg(config_.markYawTolerance) << "°), attempt " << approachAttempts_ + 1;
  if (positionOk && yawOk) {
    finishArrival();
    return;
  }
  if (++approachAttempts_ > config_.maxApproachAttempts) {
    fail(tr("final pose out of tolerance (%1 cm, %2°)")
             .arg(positionError() * 100.0, 0, 'f', 1)
             .arg(std::abs(yawError()) * 180.0 / kPi, 0, 'f', 1));
    return;
  }
  if (!positionOk) {
    approachDriving_ = false;
    setMode(Mode::FinalApproach, tr("correcting position"));
  } else {
    setMode(Mode::Aligning, tr("correcting heading"));
  }
}

void Navigator::finishArrival() {
  const NavTarget target = *target_;
  const QString summary = tr("%1 reached (%2 cm, %3°)")
                              .arg(formatTarget(target))
                              .arg(positionError() * 100.0, 0, 'f', 1)
                              .arg(std::abs(yawError()) * 180.0 / kPi, 0, 'f', 1);
  if (!target.goalName.empty()) {
    if (!client_) {
      fail(tr("not connected, cannot mark the goal"));
      return;
    }
    try {
      const double t = client_->markGoal(target.goalName);
      emit goalMarked(QString::fromStdString(target.goalName), t);
    } catch (const std::exception& e) {
      fail(tr("mark_goal failed: %1").arg(QString::fromUtf8(e.what())));
      return;
    }
  }
  VLOG_I(lcNav) << "ARRIVED: " << summary;
  target_.reset();
  clearPath();
  stopsPending_ = 3;
  setMode(Mode::Arrived, summary);
}

// ---------------------------------------------------------------------------
// Collisions
// ---------------------------------------------------------------------------

// The bumper is one switch around the whole robot: it does not say where the contact is.
// The map often does (an obstacle right next to the robot). Otherwise the contact is on the side
// the robot was driving towards; if it was not driving (e.g. a new goal while already touching
// something), the side is unknown and the escape moves find a way out by trying.
void Navigator::startRecovery() {
  if (++collisions_ > config_.maxCollisions) {
    fail(tr("too many collisions"));
    return;
  }
  contactMotion_ = lastLinear_;
  const std::optional<double> mappedContact = contactBearing(contactMotion_);
  const std::optional<double> anyContact = contactBearing();
  VLOG_I(lcNav) << "collision #" << collisions_ << " at " << pose_ << ": was driving "
                << (contactMotion_ > 0.0 ? "forwards" : (contactMotion_ < 0.0 ? "backwards" : "not at all"))
                << "; mapped obstacle on that side "
                << (mappedContact ? QStringLiteral("at %1° (robot frame)").arg(radToDeg(*mappedContact), 0, 'f', 0)
                                  : QStringLiteral("none"))
                << "; nearest mapped obstacle all round "
                << (anyContact ? QStringLiteral("at %1°").arg(radToDeg(*anyContact), 0, 'f', 0) : QStringLiteral("none"))
                << "; clearance here " << QString::number(clearanceAt(pose_), 'f', 3) << " m";
  // Hitting a mapped obstacle near an earlier hit: the map is right but the margin was not enough
  // there. Mark the contact point so the next plan keeps further away or takes another way.
  const Point2D here{pose_.x, pose_.y};
  const bool repeated = std::any_of(collisionSpots_.begin(), collisionSpots_.end(), [&](const Point2D& p) {
    return distance(p, here) < config_.repeatBumpDistance;
  });
  collisionSpots_.push_back(here);

  QString detail;
  if (mappedContact && repeated) {
    const double side = pose_.yaw + *mappedContact;
    const double reach = info_.radius + config_.contactObstacleRadius;
    costMap_.addObstacle({pose_.x + reach * std::cos(side), pose_.y + reach * std::sin(side)},
                         config_.contactObstacleRadius);
    emit costMapChanged();
    detail = tr("hit the same obstacle again, marking the contact point");
  } else if (mappedContact) {
    detail = tr("touching a mapped obstacle, moving away");
  } else if (contactMotion_ != 0.0) {
    // Unmapped obstacle where we were driving: remember it so re-planning avoids it.
    const double side = pose_.yaw + (contactMotion_ < 0.0 ? kPi : 0.0);
    const double reach = info_.radius + config_.learnedObstacleRadius;
    costMap_.addObstacle({pose_.x + reach * std::cos(side), pose_.y + reach * std::sin(side)},
                         config_.learnedObstacleRadius);
    emit costMapChanged();
    detail = tr("bumped into an unmapped obstacle, backing off");
  } else {
    detail = tr("in contact with an obstacle, getting free");
  }

  turningInPlace_ = false;
  approachDriving_ = false;
  escapeAttempt_ = 0;
  sendWheels(0.0, 0.0);
  VLOG_I(lcNav) << "recovery: " << detail;
  setMode(Mode::Recovering, detail);
  beginEscapeMove();
}

// Escape moves, in rounds of three: straight away from the contact, straight the other way,
// then turn in place (turning is always possible) so a different part of the robot faces the
// obstacle, and try again. The same as freeing the robot by hand.
void Navigator::beginEscapeMove() {
  recoverFrom_ = pose_;
  recoverTimer_.restart();
  escapeWasFree_ = false;
  switch (escapeAttempt_ % 3) {
    case 0:
      escapeTurning_ = false;
      escapeDirection_ = escapeDirection();
      break;
    case 1:
      escapeTurning_ = false;
      escapeDirection_ = -escapeDirection_;
      break;
    default: {
      // Turn towards the contact first: that brings it to the front, where reversing moves away from it.
      const int round = escapeAttempt_ / 3;
      const std::optional<double> bearing = contactBearing();
      const double firstSign = bearing && *bearing < 0.0 ? -1.0 : 1.0;
      const double sign = round % 2 == 0 ? firstSign : -firstSign;
      escapeTurning_ = true;
      escapeTurnTarget_ = wrapAngle(pose_.yaw + sign * config_.escapeTurn * (round + 1));
      break;
    }
  }
  if (escapeTurning_)
    VLOG_I(lcNav) << "escape move " << escapeAttempt_ + 1 << "/" << config_.maxEscapeAttempts + 1 << ": turn in place "
                  << QString::number(radToDeg(wrapAngle(escapeTurnTarget_ - pose_.yaw)), 'f', 0) << "° to "
                  << QString::number(radToDeg(escapeTurnTarget_), 'f', 0) << "° from " << pose_;
  else
    VLOG_I(lcNav) << "escape move " << escapeAttempt_ + 1 << "/" << config_.maxEscapeAttempts + 1 << ": straight "
                  << (escapeDirection_ > 0.0 ? "forwards" : "backwards") << " from " << pose_ << ", bumper "
                  << (bumper_ ? "pressed" : "clear");
}

void Navigator::nextEscapeMove() {
  if (!escapeTurning_)
    VLOG_I(lcNav) << "escape move " << escapeAttempt_ + 1 << " blocked after " << recoverTimer_.elapsed()
                  << " ms: moved " << QString::number(std::hypot(pose_.x - recoverFrom_.x, pose_.y - recoverFrom_.y) * 100.0, 'f', 1)
                  << " cm, bumper " << (bumper_ ? "pressed" : "clear")
                  << (escapeWasFree_ ? " (was free, then touched again)" : "");
  else
    VLOG_I(lcNav) << "escape turn done after " << recoverTimer_.elapsed() << " ms, yaw "
                  << QString::number(radToDeg(pose_.yaw), 'f', 1) << "°";
  sendWheels(0.0, 0.0);
  if (++escapeAttempt_ > config_.maxEscapeAttempts) {
    fail(tr("stuck against an obstacle - drive it free by hand, then try again"));
    return;
  }
  beginEscapeMove();
}

void Navigator::recover() {
  if (escapeTurning_) {
    const double error = wrapAngle(escapeTurnTarget_ - pose_.yaw);
    if (std::abs(error) <= config_.escapeTurnTolerance || recoverTimer_.elapsed() > config_.recoverTimeoutMs) {
      nextEscapeMove();
      return;
    }
    sendWheels(0.0, turnCommand(error, config_.turnGain, config_.minTurnRate, config_.alignMaxRate));
    return;
  }

  const double moved = std::hypot(pose_.x - recoverFrom_.x, pose_.y - recoverFrom_.y);
  const qint64 elapsed = recoverTimer_.elapsed();
  if (!bumper_) {
    escapeWasFree_ = true;
    if (moved >= config_.backoffDistance || elapsed > config_.recoverTimeoutMs) {
      VLOG_I(lcNav) << "free after escape move " << escapeAttempt_ + 1 << ": moved "
                    << QString::number(moved * 100.0, 'f', 1) << " cm in " << elapsed << " ms, now at " << pose_;
      sendWheels(0.0, 0.0);
      replan(tr("avoiding the obstacle"));
      return;
    }
  } else if (escapeWasFree_ || (elapsed > config_.escapeMoveMs && moved < config_.escapeMinProgress) ||
             elapsed > config_.recoverTimeoutMs) {
    nextEscapeMove();  // blocked this way, or touched something else on the way out
    return;
  }
  sendWheels(escapeDirection_ * config_.backoffSpeed, 0.0);
  if (tickLog_.ready())
    VLOG_D(lcNavTick) << "escape: " << (escapeDirection_ > 0.0 ? "forwards" : "backwards") << " moved "
                      << QString::number(moved * 100.0, 'f', 1) << " cm in " << elapsed << " ms, bumper "
                      << (bumper_ ? "pressed" : "clear") << ", pose " << pose_;
}

std::optional<double> Navigator::contactBearing(double motion) const {
  if (!costMap_.isValid()) return std::nullopt;
  int col = 0;
  int row = 0;
  if (!costMap_.worldToCell({pose_.x, pose_.y}, col, row)) return std::nullopt;
  const double reach = info_.radius + config_.contactSearchMargin;
  const int cells = static_cast<int>(std::ceil(reach / costMap_.meta().resolution));
  double nearest = reach;
  std::optional<double> bearing;
  for (int r = row - cells; r <= row + cells; ++r) {
    for (int c = col - cells; c <= col + cells; ++c) {
      if (!costMap_.contains(c, r) || !costMap_.isOccupied(c, r)) continue;
      const Point2D p = costMap_.cellCenter(c, r);
      const double d = std::hypot(p.x - pose_.x, p.y - pose_.y);
      if (d > nearest) continue;
      const double b = wrapAngle(std::atan2(p.y - pose_.y, p.x - pose_.x) - pose_.yaw);
      if ((motion > 0.0 && std::abs(b) >= kPi / 2) || (motion < 0.0 && std::abs(b) <= kPi / 2)) continue;
      nearest = d;
      bearing = b;
    }
  }
  return bearing;
}

double Navigator::escapeDirection() const {
  // Hit while driving: the contact is where we were going (until an escape turn changes that).
  if (contactMotion_ != 0.0 && escapeAttempt_ < 3) return contactMotion_ > 0.0 ? -1.0 : 1.0;
  if (const std::optional<double> bearing = contactBearing()) return std::abs(*bearing) < kPi / 2 ? -1.0 : 1.0;
  return -1.0;  // unknown: the robot mostly drives forwards, so back up first
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool Navigator::planPath() {
  const PlanResult plan = PathPlanner::plan(costMap_, {pose_.x, pose_.y}, {target_->x, target_->y});
  if (!plan.ok) {
    for (std::size_t i = 0; i < costMap_.learnedObstacles().size(); ++i) {
      const LearnedObstacle& o = costMap_.learnedObstacles()[i];
      VLOG_W(lcNav) << "  learned obstacle #" << i + 1 << " at " << o.centre << ", "
                    << QString::number(distance(o.centre, {pose_.x, pose_.y}), 'f', 2) << " m from the robot";
    }
    fail(tr("planning failed: %1").arg(QString::fromStdString(plan.error)));
    return false;
  }
  VLOG_I(lcNav) << "following a " << QString::number(plan.length, 'f', 2) << " m path with " << plan.waypoints.size()
                << " waypoints";
  path_ = plan.waypoints;
  segment_ = 0;
  turningInPlace_ = false;
  approachDriving_ = false;
  emit pathChanged(path_);
  return true;
}

void Navigator::replan(const QString& reason) {
  VLOG_I(lcNav) << "re-plan " << replans_ + 1 << "/" << config_.maxReplans << " (" << reason << ") from " << pose_;
  if (++replans_ > config_.maxReplans) {
    fail(tr("gave up after %1 re-plans").arg(config_.maxReplans));
    return;
  }
  if (!planPath()) return;
  setMode(Mode::Following, tr("re-planned: %1").arg(reason));
}

void Navigator::fail(const QString& reason) {
  VLOG_W(lcNav) << "FAILED" << (target_ ? QStringLiteral(" to reach %1").arg(formatTarget(*target_)) : QString())
                << ": " << reason << " (at " << pose_ << ", bumper " << (bumper_ ? "pressed" : "clear") << ", "
                << collisions_ << " collision(s), " << replans_ << " re-plan(s))";
  sendWheels(0.0, 0.0);
  stopsPending_ = 3;
  target_.reset();
  clearPath();
  setMode(Mode::Failed, reason);
}

void Navigator::setMode(Mode mode, const QString& detail) {
  if (mode != mode_ || !detail.isEmpty())
    VLOG_I(lcNav) << "mode " << toString(mode_) << " -> " << toString(mode) << (detail.isEmpty() ? QString() : QStringLiteral(": ") + detail);
  mode_ = mode;
  emit modeChanged(mode, detail);
}

Point2D Navigator::pointAlongPath(std::size_t segment, const Point2D& from, double length) const {
  Point2D current = from;
  double left = length;
  for (std::size_t i = segment; i + 1 < path_.size(); ++i) {
    const Point2D& next = path_[i + 1];
    const double d = distance(current, next);
    if (d > 1e-9 && d >= left) {
      const double t = left / d;
      return {current.x + t * (next.x - current.x), current.y + t * (next.y - current.y)};
    }
    left -= d;
    current = next;
  }
  return path_.back();
}

double Navigator::positionError() const {
  return target_ ? std::hypot(target_->x - pose_.x, target_->y - pose_.y) : 0.0;
}

double Navigator::yawError() const {
  return target_ && target_->yaw ? wrapAngle(*target_->yaw - pose_.yaw) : 0.0;
}

double Navigator::clearanceAt(const Pose2D& pose) const { return costMap_.clearanceAt({pose.x, pose.y}); }

double Navigator::tightness(double clearance) const {
  const double hard = costMap_.hardClearance();
  if (config_.tightClearance <= hard) return 0.0;
  return std::clamp((config_.tightClearance - clearance) / (config_.tightClearance - hard), 0.0, 1.0);
}

void Navigator::sendWheels(double linear, double angular) {
  lastLinear_ = linear;
  lastAngular_ = angular;
  if (!client_) return;
  const double halfBase = 0.5 * info_.wheelBase;
  double left = (linear - angular * halfBase) / info_.wheelRadius;
  double right = (linear + angular * halfBase) / info_.wheelRadius;
  const double peak = std::max(std::abs(left), std::abs(right));
  if (peak > info_.maxWheelSpeed) {  // scale both wheels to keep the curvature
    left *= info_.maxWheelSpeed / peak;
    right *= info_.maxWheelSpeed / peak;
  }
  client_->sendCommand(protocol::wheelsCommand(left, right));
}

void Navigator::clearPath() {
  if (path_.empty()) return;
  path_.clear();
  segment_ = 0;
  emit pathChanged(path_);
}

}  // namespace vacuum
