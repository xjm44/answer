#pragma once

// JSON <-> data type conversion for the simulator's wire format.
// Parsers are lenient about missing optional fields and throw nlohmann::json::exception
// on malformed required ones.

#include <optional>
#include <vector>

#include <nlohmann/json.hpp>

#include "core/common/Types.h"

namespace vacuum::protocol {

using Json = nlohmann::json;

Pose2D poseFromJson(const Json& j);
Goal goalFromJson(const Json& j);
std::vector<Goal> goalsFromJson(const Json& j);
SceneInfo sceneInfoFromJson(const Json& j);
SceneLoadResult sceneLoadResultFromJson(const Json& j);
RobotInfo robotInfoFromJson(const Json& j);
MapMeta mapMetaFromJson(const Json& j);
RobotState robotStateFromJson(const Json& j);

// Commands (port 5556).
Json wheelsCommand(double left, double right);
Json gotoCommand(int id, double x, double y, std::optional<double> yaw);
Json cancelCommand(int id);

}  // namespace vacuum::protocol
