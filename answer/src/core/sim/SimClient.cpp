#include "core/sim/SimClient.h"

#include <iterator>
#include <mutex>

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <QElapsedTimer>

#include "core/common/Log.h"
#include "core/sim/Protocol.h"
#include "core/sim/ZmqContext.h"

namespace vacuum {

using Json = nlohmann::json;

namespace {

// JSON for a log line, cut to a readable length.
QString shortJson(const Json& json, int maxChars = 300) {
  QString text = QString::fromStdString(json.dump());
  if (text.size() > maxChars) text = text.left(maxChars) + QStringLiteral("... (%1 chars)").arg(text.size());
  return text;
}

}  // namespace

zmq::context_t& sharedZmqContext() {
  static zmq::context_t context;
  return context;
}

struct SimClient::Impl {
  std::string host;
  int timeoutMs;
  std::mutex rpcMutex;  // REQ sockets are strictly send/recv alternating
  std::unique_ptr<zmq::socket_t> rpc;
  std::mutex cmdMutex;  // ZeroMQ sockets are not thread-safe
  zmq::socket_t cmd{sharedZmqContext(), zmq::socket_type::push};
  std::int64_t commandsSent = 0;
  std::int64_t commandsFailed = 0;
  LogThrottle commandLog{5000};

  Impl(std::string h, int timeout) : host(std::move(h)), timeoutMs(timeout) {
    openRpc();
    cmd.set(zmq::sockopt::linger, 0);
    cmd.set(zmq::sockopt::sndhwm, 10);
    cmd.set(zmq::sockopt::immediate, 1);  // never queue commands for a simulator that is not there
    cmd.connect(endpoint(host, ports::kCommand));
  }

  // (Re)creates the REQ socket. Needed after a timeout, when the socket is stuck awaiting a reply.
  void openRpc() {
    if (rpc) VLOG_W(lcSim) << "RPC socket re-created (the previous request got no reply)";
    rpc = std::make_unique<zmq::socket_t>(sharedZmqContext(), zmq::socket_type::req);
    rpc->set(zmq::sockopt::linger, 0);
    rpc->set(zmq::sockopt::rcvtimeo, timeoutMs);
    rpc->set(zmq::sockopt::sndtimeo, timeoutMs);
    rpc->connect(endpoint(host, ports::kRpc));
  }
};

SimClient::SimClient(std::string host, int rpcTimeoutMs)
    : impl_(std::make_unique<Impl>(std::move(host), rpcTimeoutMs)) {
  VLOG_I(lcSim) << "client for " << impl_->host.c_str() << ": RPC " << endpoint(impl_->host, ports::kRpc).c_str()
                << " (timeout " << rpcTimeoutMs << " ms), commands " << endpoint(impl_->host, ports::kCommand).c_str();
}

SimClient::~SimClient() = default;

const std::string& SimClient::host() const { return impl_->host; }

std::string SimClient::endpoint(const std::string& host, int port) {
  return "tcp://" + host + ":" + std::to_string(port);
}

Json SimClient::call(const std::string& method, const Json& params, std::vector<std::string>* blobs) {
  std::lock_guard<std::mutex> lock(impl_->rpcMutex);
  const std::string request = Json{{"method", method}, {"params", params}}.dump();
  QElapsedTimer clock;
  clock.start();
  VLOG_D(lcSim) << "RPC -> " << method.c_str() << " " << shortJson(params);
  auto failed = [&](const std::string& what) {
    VLOG_W(lcSim) << "RPC " << method.c_str() << " failed after " << clock.elapsed() << " ms: " << what.c_str();
    return SimError("'" + method + "': " + what);
  };

  std::vector<zmq::message_t> frames;
  try {
    if (!impl_->rpc->send(zmq::buffer(request), zmq::send_flags::none)) {
      impl_->openRpc();
      throw failed("simulator not reachable at " + impl_->host);
    }
    if (!zmq::recv_multipart(*impl_->rpc, std::back_inserter(frames)) || frames.empty()) {
      impl_->openRpc();
      throw failed("no reply from simulator within " + std::to_string(impl_->timeoutMs) + " ms");
    }
  } catch (const zmq::error_t& e) {
    impl_->openRpc();
    throw failed(e.what());
  }

  const Json reply = Json::parse(frames[0].to_string(), nullptr, /*allow_exceptions=*/false);
  if (reply.is_discarded() || !reply.is_object()) throw failed("malformed reply");
  if (!reply.value("ok", false)) {
    const auto error = reply.find("error");
    throw failed(error != reply.end() && error->is_string() ? error->get<std::string>() : "unknown error");
  }
  if (blobs) {
    for (std::size_t i = 1; i < frames.size(); ++i) blobs->emplace_back(frames[i].data<char>(), frames[i].size());
  }
  const auto result = reply.find("result");
  const Json value = result != reply.end() ? *result : Json();
  VLOG_D(lcSim) << "RPC <- " << method.c_str() << " ok in " << clock.elapsed() << " ms: " << shortJson(value)
                << (frames.size() > 1 ? QStringLiteral(" + %1 binary frame(s), %2 bytes")
                                            .arg(frames.size() - 1)
                                            .arg(frames.size() > 1 ? frames[1].size() : 0)
                                      : QString());
  return value;
}

double SimClient::ping() { return call("ping").value("t", 0.0); }

std::vector<SceneInfo> SimClient::listScenes() {
  std::vector<SceneInfo> scenes;
  for (const Json& item : call("list_scenes")) scenes.push_back(protocol::sceneInfoFromJson(item));
  return scenes;
}

SceneLoadResult SimClient::loadScene(const std::string& id, std::optional<int> seed) {
  Json params{{"id", id}};
  if (seed) params["seed"] = *seed;
  VLOG_I(lcSim) << "load_scene " << id.c_str() << (seed ? QStringLiteral(" seed %1").arg(*seed) : QStringLiteral(" (random start)"));
  const SceneLoadResult result = protocol::sceneLoadResultFromJson(call("load_scene", params));
  VLOG_I(lcSim) << "scene " << result.id.c_str() << " loaded: start " << result.start << ", " << result.goals.size()
                << " goals";
  return result;
}

SceneLoadResult SimClient::reset(std::optional<int> seed) {
  Json params = Json::object();
  if (seed) params["seed"] = *seed;
  VLOG_I(lcSim) << "reset" << (seed ? QStringLiteral(" seed %1").arg(*seed) : QStringLiteral(" (random start)"));
  const SceneLoadResult result = protocol::sceneLoadResultFromJson(call("reset", params));
  VLOG_I(lcSim) << "scene " << result.id.c_str() << " reset: start " << result.start;
  return result;
}

std::vector<Goal> SimClient::getGoals() { return protocol::goalsFromJson(call("get_goals")); }

OccupancyMap SimClient::getMap() {
  std::vector<std::string> blobs;
  const Json meta = call("get_map", Json::object(), &blobs);
  if (blobs.empty()) throw SimError("'get_map': reply has no PNG frame");
  try {
    return OccupancyMap::fromPng(protocol::mapMetaFromJson(meta), blobs.front());
  } catch (const std::exception& e) {
    throw SimError(std::string("'get_map': ") + e.what());
  }
}

RobotInfo SimClient::getRobotInfo() { return protocol::robotInfoFromJson(call("get_robot_info")); }

Json SimClient::getCameraInfo() { return call("get_camera_info"); }

double SimClient::markGoal(const std::string& name) {
  VLOG_I(lcSim) << "mark_goal '" << name.c_str() << "'";
  const double t = call("mark_goal", Json{{"name", name}}).value("t", 0.0);
  VLOG_I(lcSim) << "goal '" << name.c_str() << "' marked at t = " << QString::number(t, 'f', 2) << " s";
  return t;
}

bool SimClient::sendCommand(const Json& command) {
  const std::string payload = command.dump();
  std::lock_guard<std::mutex> lock(impl_->cmdMutex);
  bool sent = false;
  try {
    sent = impl_->cmd.send(zmq::buffer(payload), zmq::send_flags::dontwait).has_value();
  } catch (const zmq::error_t& e) {
    VLOG_W(lcSim) << "command send error: " << e.what();
  }
  ++impl_->commandsSent;
  if (!sent) ++impl_->commandsFailed;
  if (command.value("type", std::string()) != "wheels") {
    VLOG_I(lcSim) << "command " << payload.c_str() << (sent ? "" : " NOT SENT (simulator not connected)");
  } else if (impl_->commandLog.ready()) {
    VLOG_D(lcSim) << "wheel commands: " << impl_->commandsSent << " sent, " << impl_->commandsFailed
                  << " could not be queued; latest " << payload.c_str();
  }
  return sent;
}

}  // namespace vacuum
