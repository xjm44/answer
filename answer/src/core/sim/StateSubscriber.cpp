#include "core/sim/StateSubscriber.h"

#include <iterator>
#include <vector>

#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <QElapsedTimer>

#include "core/common/Log.h"
#include "core/sim/Protocol.h"
#include "core/sim/ZmqContext.h"

namespace vacuum {

namespace {
constexpr int kPollTimeoutMs = 100;  // how quickly the thread notices a stop request
constexpr int kSummaryIntervalMs = 2000;
}

StateSubscriber::StateSubscriber(QString endpoint, int staleAfterMs, QObject* parent)
    : QThread(parent), endpoint_(std::move(endpoint)), staleAfterMs_(staleAfterMs) {
  qRegisterMetaType<vacuum::RobotState>("vacuum::RobotState");
  setObjectName(QStringLiteral("state"));
}

StateSubscriber::~StateSubscriber() {
  requestInterruption();
  wait();
}

void StateSubscriber::run() {
  try {
    zmq::socket_t socket(sharedZmqContext(), zmq::socket_type::sub);
    socket.set(zmq::sockopt::linger, 0);
    socket.set(zmq::sockopt::rcvtimeo, kPollTimeoutMs);
    socket.set(zmq::sockopt::subscribe, "");
    socket.connect(endpoint_.toStdString());
    VLOG_I(lcState) << "subscribed to " << endpoint_;

    bool active = false;
    QElapsedTimer sinceLastMessage;
    sinceLastMessage.start();

    // Statistics for the periodic summary.
    QElapsedTimer summaryClock;
    summaryClock.start();
    std::int64_t received = 0;
    std::int64_t lost = 0;
    std::int64_t corrupt = 0;
    std::int64_t lastSeq = -1;
    std::string lastScene;
    bool lastBumper = false;
    RobotState last;

    while (!isInterruptionRequested()) {
      std::vector<zmq::message_t> frames;
      if (!zmq::recv_multipart(socket, std::back_inserter(frames)) || frames.empty()) {
        if (active && sinceLastMessage.elapsed() > staleAfterMs_) {
          active = false;
          VLOG_W(lcState) << "no state for " << sinceLastMessage.elapsed() << " ms: stream inactive";
          emit streamActiveChanged(false);
        }
        continue;
      }

      const auto json = protocol::Json::parse(frames[0].to_string(), nullptr, /*allow_exceptions=*/false);
      if (json.is_discarded() || !json.is_object()) {  // drop corrupt messages, keep streaming
        ++corrupt;
        VLOG_W(lcState) << "corrupt state message dropped (" << frames[0].size() << " bytes)";
        continue;
      }
      RobotState state;
      try {
        state = protocol::robotStateFromJson(json);
      } catch (const std::exception& e) {
        ++corrupt;
        VLOG_W(lcState) << "unreadable state message dropped: " << e.what();
        continue;
      }

      ++received;
      if (lastSeq >= 0 && state.seq > lastSeq + 1) lost += state.seq - lastSeq - 1;
      if (lastSeq >= 0 && state.seq < lastSeq) VLOG_I(lcState) << "seq went back " << lastSeq << " -> " << state.seq;
      lastSeq = state.seq;
      if (state.scene != lastScene) {
        VLOG_I(lcState) << "scene in stream: '" << lastScene.c_str() << "' -> '" << state.scene.c_str() << "' at t = "
                        << QString::number(state.t, 'f', 2);
        lastScene = state.scene;
      }
      if (state.bumper != lastBumper) {
        VLOG_I(lcState) << (state.bumper ? "BUMPER PRESSED" : "bumper released") << " at t = "
                        << QString::number(state.t, 'f', 2) << ", raw pose " << state.pose;
        lastBumper = state.bumper;
      }
      if (last.gotoId != state.gotoId || last.gotoStatus != state.gotoStatus)
        VLOG_I(lcState) << "goto status: " << (state.gotoId ? QString::number(*state.gotoId) : QStringLiteral("-"))
                        << " " << toString(state.gotoStatus);
      last = state;

      sinceLastMessage.restart();
      if (!active) {
        active = true;
        VLOG_I(lcState) << "stream active (t = " << QString::number(state.t, 'f', 2) << ", scene '"
                        << state.scene.c_str() << "')";
        emit streamActiveChanged(true);
      }
      if (summaryClock.elapsed() >= kSummaryIntervalMs) {
        VLOG_D(lcState) << QString::number(received * 1000.0 / summaryClock.elapsed(), 'f', 1) << " Hz, " << lost
                        << " lost, " << corrupt << " corrupt | t " << QString::number(state.t, 'f', 2) << " raw "
                        << state.pose << " ticks L " << state.leftTicks << " R " << state.rightTicks << " bumper "
                        << (state.bumper ? 1 : 0);
        summaryClock.restart();
        received = 0;
        lost = 0;
        corrupt = 0;
      }
      emit stateReceived(state);
    }
    VLOG_I(lcState) << "stream thread stopped";
  } catch (const zmq::error_t& e) {
    VLOG_E(lcState) << "stream error: " << e.what();
    emit streamError(QString::fromUtf8(e.what()));
  }
}

}  // namespace vacuum
