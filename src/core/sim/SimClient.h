#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "core/common/Types.h"
#include "core/map/OccupancyMap.h"

namespace vacuum {

namespace ports {
inline constexpr int kRpc = 5555;
inline constexpr int kCommand = 5556;
inline constexpr int kState = 5557;
inline constexpr int kCamera = 5558;
}  // namespace ports

// Error reported by the simulator, or a transport failure (timeout, malformed reply).
class SimError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Request/reply access to the simulator (port 5555) plus the command channel (port 5556).
//
// RPC calls block for at most `rpcTimeoutMs`. After a timeout the REQ socket is
// recreated, so one lost reply does not wedge the client. Calls are serialised
// internally, so the client may be shared between threads.
class SimClient {
 public:
  explicit SimClient(std::string host, int rpcTimeoutMs = 3000);
  ~SimClient();
  SimClient(const SimClient&) = delete;
  SimClient& operator=(const SimClient&) = delete;

  const std::string& host() const;
  static std::string endpoint(const std::string& host, int port);

  // Raw RPC. Returns "result"; extra binary frames (e.g. the map PNG) go to `blobs`.
  nlohmann::json call(const std::string& method, const nlohmann::json& params = nlohmann::json::object(),
                      std::vector<std::string>* blobs = nullptr);

  double ping();
  std::vector<SceneInfo> listScenes();
  SceneLoadResult loadScene(const std::string& id, std::optional<int> seed = std::nullopt);
  SceneLoadResult reset(std::optional<int> seed = std::nullopt);
  std::vector<Goal> getGoals();
  OccupancyMap getMap();
  RobotInfo getRobotInfo();
  nlohmann::json getCameraInfo();
  double markGoal(const std::string& name);

  // Fire-and-forget command. Returns false if it could not be queued (simulator not connected).
  bool sendCommand(const nlohmann::json& command);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vacuum
