#include "core/sim/Protocol.h"

namespace vacuum::protocol {

Pose2D poseFromJson(const Json& j) {
  Pose2D p;
  p.x = j.value("x", 0.0);
  p.y = j.value("y", 0.0);
  p.yaw = j.value("yaw", 0.0);
  return p;
}

Goal goalFromJson(const Json& j) {
  Goal g;
  g.name = j.at("name").get<std::string>();
  const Json& pose = j.at("pose");  // [x, y] or [x, y, yaw]
  g.x = pose.at(0).get<double>();
  g.y = pose.at(1).get<double>();
  if (pose.size() > 2 && pose[2].is_number()) g.yaw = pose[2].get<double>();
  return g;
}

std::vector<Goal> goalsFromJson(const Json& j) {
  std::vector<Goal> goals;
  for (const Json& item : j) goals.push_back(goalFromJson(item));
  return goals;
}

SceneInfo sceneInfoFromJson(const Json& j) {
  SceneInfo s;
  s.id = j.at("id").get<std::string>();
  s.name = j.value("name", std::string());
  s.difficulty = j.value("difficulty", std::string());
  return s;
}

SceneLoadResult sceneLoadResultFromJson(const Json& j) {
  SceneLoadResult r;
  r.id = j.value("id", std::string());
  r.start = poseFromJson(j.at("start"));
  r.goals = goalsFromJson(j.at("goals"));
  return r;
}

RobotInfo robotInfoFromJson(const Json& j) {
  RobotInfo r;  // defaults are the documented values
  r.radius = j.value("radius", r.radius);
  r.height = j.value("height", r.height);
  r.wheelBase = j.value("wheel_base", r.wheelBase);
  r.wheelRadius = j.value("wheel_radius", r.wheelRadius);
  r.maxLinearSpeed = j.value("max_linear_speed", r.maxLinearSpeed);
  r.maxAngularSpeed = j.value("max_angular_speed", r.maxAngularSpeed);
  r.maxLinearAccel = j.value("max_linear_accel", r.maxLinearAccel);
  r.maxAngularAccel = j.value("max_angular_accel", r.maxAngularAccel);
  r.maxWheelSpeed = j.value("max_wheel_speed", r.maxWheelSpeed);
  r.ticksPerRev = j.value("ticks_per_rev", r.ticksPerRev);
  r.controlRate = j.value("control_rate", r.controlRate);
  r.watchdog = j.value("watchdog", r.watchdog);
  r.stateRate = j.value("state_rate", r.stateRate);
  return r;
}

MapMeta mapMetaFromJson(const Json& j) {
  MapMeta m;
  m.resolution = j.at("resolution").get<double>();
  const Json& origin = j.at("origin");
  m.origin = {origin.at(0).get<double>(), origin.at(1).get<double>()};
  m.width = j.at("width").get<int>();
  m.height = j.at("height").get<int>();
  return m;
}

RobotState robotStateFromJson(const Json& j) {
  RobotState s;
  s.t = j.value("t", 0.0);
  s.seq = j.value("seq", std::int64_t{0});
  if (auto it = j.find("scene"); it != j.end() && it->is_string()) s.scene = it->get<std::string>();
  if (auto it = j.find("pose"); it != j.end() && it->is_object()) s.pose = poseFromJson(*it);
  if (auto it = j.find("wheels"); it != j.end() && it->is_object()) {
    s.leftTicks = it->value("left_ticks", std::int64_t{0});
    s.rightTicks = it->value("right_ticks", std::int64_t{0});
  }
  if (auto it = j.find("bumper"); it != j.end() && it->is_boolean()) s.bumper = it->get<bool>();
  // "goto": {"id": null, "status": null} until the first goto after a load/reset.
  if (auto it = j.find("goto"); it != j.end() && it->is_object()) {
    if (auto id = it->find("id"); id != it->end() && id->is_number_integer()) s.gotoId = id->get<int>();
    if (auto st = it->find("status"); st != it->end() && st->is_string())
      s.gotoStatus = gotoStatusFromString(st->get<std::string>());
  }
  return s;
}

Json wheelsCommand(double left, double right) {
  return {{"type", "wheels"}, {"left", left}, {"right", right}};
}

Json gotoCommand(int id, double x, double y, std::optional<double> yaw) {
  Json cmd{{"type", "goto"}, {"id", id}, {"x", x}, {"y", y}};
  if (yaw) cmd["yaw"] = *yaw;
  return cmd;
}

Json cancelCommand(int id) {
  return {{"type", "cancel"}, {"id", id}};
}

}  // namespace vacuum::protocol
