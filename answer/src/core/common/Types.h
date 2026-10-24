#pragma once

// Plain data types shared by the whole application. No Qt, no ZeroMQ.
// Units are SI (metres, radians, seconds); frames follow the simulator spec.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace vacuum {

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;  // counter-clockwise from +x, (-pi, pi]
};

struct Goal {
  std::string name;
  double x = 0.0;
  double y = 0.0;
  std::optional<double> yaw;  // only some goals constrain the final heading
};

struct SceneInfo {
  std::string id;
  std::string name;
  std::string difficulty;  // "easy" | "advanced"
};

// Result of load_scene / reset.
struct SceneLoadResult {
  std::string id;
  Pose2D start;
  std::vector<Goal> goals;
};

struct RobotInfo {
  double radius = 0.17;
  double height = 0.09;
  double wheelBase = 0.23;
  double wheelRadius = 0.035;
  double maxLinearSpeed = 0.35;
  double maxAngularSpeed = 2.0;
  double maxLinearAccel = 1.0;
  double maxAngularAccel = 6.0;
  double maxWheelSpeed = 10.0;
  int ticksPerRev = 1024;
  double controlRate = 100.0;
  double watchdog = 0.25;
  double stateRate = 50.0;
};

// Map header from get_map (spec section 7).
struct MapMeta {
  double resolution = 0.0;  // metres per cell
  Point2D origin;           // world position of the bottom-left corner of the image
  int width = 0;            // columns
  int height = 0;           // rows
};

enum class GotoStatus { None, Running, Reached, Bumped, Cancelled, Preempted, Unknown };

const char* toString(GotoStatus status);
GotoStatus gotoStatusFromString(const std::string& text);

// One message of the state stream (port 5557).
struct RobotState {
  double t = 0.0;  // simulation time since last load/reset
  std::int64_t seq = 0;
  std::string scene;
  Pose2D pose;
  std::int64_t leftTicks = 0;
  std::int64_t rightTicks = 0;
  bool bumper = false;
  std::optional<int> gotoId;
  GotoStatus gotoStatus = GotoStatus::None;
};

}  // namespace vacuum
