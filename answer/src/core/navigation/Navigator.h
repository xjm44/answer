#pragma once

#include <optional>
#include <string>
#include <vector>

#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QTimer>

#include "core/common/Geometry.h"
#include "core/common/Log.h"
#include "core/common/Types.h"
#include "core/map/CostMap.h"

namespace vacuum {

class SimClient;

struct NavTarget {
  double x = 0.0;
  double y = 0.0;
  std::optional<double> yaw;
  std::string goalName;  // non-empty: call mark_goal with it on arrival
};

struct NavigatorConfig {
  int tickMs = 50;              // control rate 20 Hz (watchdog is 250 ms)
  int stateTimeoutMs = 500;     // stop driving if the state stream goes quiet

  // Path following (pure pursuit)
  double cruiseSpeed = 0.30;    // m/s (limit 0.35)
  double deceleration = 0.35;   // m/s^2 used to slow down before the goal
  double lookaheadMin = 0.18;   // m
  double lookaheadMax = 0.35;   // m
  double maxAngularSpeed = 1.5;  // rad/s (limit 2.0)
  double turnGain = 2.5;        // rad/s per rad of heading error
  double turnInPlaceEnter = degToRad(50.0);
  double turnInPlaceExit = degToRad(15.0);
  double offPathDistance = 0.30;  // replan when further than this from the path

  // Tight spaces: below `tightClearance` (robot centre to the nearest obstacle, at the robot or on the
  // next `tightPreview` metres of path) speed and look-ahead shrink linearly towards the tight values
  // at the hard clearance limit, so the robot tracks the path to within millimetres.
  double tightClearance = 0.28;
  double tightPreview = 0.30;
  double tightSpeed = 0.08;
  double tightLookahead = 0.10;

  // Final approach and alignment
  double finalApproachDistance = 0.15;  // switch from path following to point approach
  double approachSpeed = 0.10;
  double approachGain = 0.8;           // m/s per m of remaining distance
  double approachAlignTolerance = degToRad(12.0);
  double reverseDistance = 0.12;       // goal behind and closer than this: back up instead of turning
  double positionTolerance = 0.015;    // m, aim
  double yawTolerance = degToRad(1.0);
  double minTurnRate = 0.12;           // rad/s, avoids stalling near zero error
  double alignMaxRate = 1.0;

  // Arrival check before mark_goal (the simulator accepts 5 cm / 5 deg)
  int settleMs = 500;
  double markPositionTolerance = 0.03;
  double markYawTolerance = degToRad(3.0);
  int maxApproachAttempts = 6;

  // Collision recovery: get free of the contact, then re-plan
  double learnedObstacleRadius = 0.06;
  double contactSearchMargin = 0.05;  // a mapped obstacle this close to the robot's edge explains a bump
  double backoffDistance = 0.08;      // move this far once free, so re-planning starts with some room
  double backoffSpeed = 0.08;
  int escapeMoveMs = 800;             // a straight escape move that has not got going by then is blocked
  double escapeMinProgress = 0.01;    // m
  int recoverTimeoutMs = 3000;        // longest single escape move
  double escapeTurn = degToRad(35.0);  // turn in place between escape attempts (grows each round)
  double escapeTurnTolerance = degToRad(3.0);
  int maxEscapeAttempts = 12;
  double repeatBumpDistance = 0.10;     // a second bump this close to an earlier one: mark the contact point
  double contactObstacleRadius = 0.03;  // obstacle added at that contact point
  int maxCollisions = 12;
  int maxReplans = 25;
};

// Drives the robot: plans on the cost map, follows the path with wheel commands,
// approaches and aligns precisely, verifies and marks the goal. Bumper hits add an
// obstacle to the cost map, the robot backs off and re-plans.
//
// It is also the single owner of the command channel: manual driving goes through it
// too, so manual and autonomous commands never fight. Runs on the GUI thread (QTimer).
class Navigator : public QObject {
  Q_OBJECT

 public:
  enum class Mode { Idle, Manual, Following, FinalApproach, Aligning, Settling, Recovering, Arrived, Failed };
  static const char* toString(Mode mode);

  explicit Navigator(QObject* parent = nullptr);

  void setClient(SimClient* client);  // not owned; nullptr detaches
  void setRobotInfo(const RobotInfo& info) { info_ = info; }
  void setMap(const OccupancyMap& map);  // stops, rebuilds the cost map, forgets learned obstacles
  const CostMap& costMap() const { return costMap_; }
  const NavigatorConfig& config() const { return config_; }

  // Feed every state message with the filtered pose.
  void onState(const RobotState& state, const Pose2D& estimate);

  bool navigateTo(const NavTarget& target);
  void driveManual(double linear, double angular);  // (0, 0) releases
  void stop();

  Mode mode() const { return mode_; }
  bool isBusy() const;  // driving autonomously or manually
  const std::optional<NavTarget>& target() const { return target_; }

 signals:
  void modeChanged(vacuum::Navigator::Mode mode, const QString& detail);
  void pathChanged(const std::vector<vacuum::Point2D>& path);
  void costMapChanged();
  void goalMarked(const QString& name, double simTime);

 private:
  void tick();
  void setMode(Mode mode, const QString& detail = QString());
  bool planPath();
  void replan(const QString& reason);
  void fail(const QString& reason);

  void followPath();
  void finalApproach();
  void afterPositionReached();
  void align();
  void beginSettling();
  void settle();
  void finishArrival();
  void startRecovery();
  void recover();
  void beginEscapeMove();
  void nextEscapeMove();
  // Robot frame bearing of the nearest mapped obstacle touching the robot. `motion` > 0 only looks
  // in the front half, < 0 only in the rear half, 0 all round.
  std::optional<double> contactBearing(double motion = 0.0) const;
  double escapeDirection() const;                // +1 forward / -1 reverse: away from the contact
  double clearanceAt(const Pose2D& pose) const;  // metres to the nearest obstacle on the cost map
  double tightness(double clearance) const;      // 0 in open space .. 1 at the hard clearance limit

  Point2D pointAlongPath(std::size_t segment, const Point2D& from, double length) const;
  double positionError() const;
  double yawError() const;  // 0 if the target has no yaw
  void sendWheels(double linear, double angular);
  void clearPath();

  NavigatorConfig config_;
  SimClient* client_ = nullptr;
  RobotInfo info_;
  CostMap costMap_;
  QTimer timer_;

  Mode mode_ = Mode::Idle;
  std::optional<NavTarget> target_;
  std::vector<Point2D> path_;
  std::size_t segment_ = 0;
  bool turningInPlace_ = false;
  bool approachDriving_ = false;
  bool approachReverse_ = false;
  int approachAttempts_ = 0;
  int collisions_ = 0;
  std::vector<Point2D> collisionSpots_;  // robot positions of this navigation's collisions
  int replans_ = 0;

  bool haveState_ = false;
  QElapsedTimer stateAge_;
  Pose2D pose_;
  bool bumper_ = false;

  double manualLinear_ = 0.0;
  double manualAngular_ = 0.0;
  double lastLinear_ = 0.0;
  int stopsPending_ = 0;  // zero commands still to send after stopping

  LogThrottle tickLog_{200};   // control loop lines: 5 per second
  LogThrottle staleLog_{1000};
  double lastAngular_ = 0.0;

  QElapsedTimer settleTimer_;
  QElapsedTimer recoverTimer_;  // current escape move
  Pose2D recoverFrom_;          // pose at the start of the current escape move
  double contactMotion_ = 0.0;  // linear command when the contact was detected
  int escapeAttempt_ = 0;
  bool escapeTurning_ = false;
  double escapeTurnTarget_ = 0.0;
  double escapeDirection_ = -1.0;
  bool escapeWasFree_ = false;  // bumper cleared during the current move
};

}  // namespace vacuum
