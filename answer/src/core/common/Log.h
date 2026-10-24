#pragma once

// Logging for the whole application.
//
// Everything goes through Qt's logging (qCDebug / qCInfo / qCWarning) in the categories below.
// installLogHandler() sends it to the console and to a log file. Categories can be switched off
// at run time with QT_LOGGING_RULES, e.g.
//   set QT_LOGGING_RULES=vacuum.nav.tick.debug=false;vacuum.state.debug=false

#include <QDebug>
#include <QElapsedTimer>
#include <QLoggingCategory>
#include <QString>

#include "core/common/Types.h"

Q_DECLARE_LOGGING_CATEGORY(lcApp)      // vacuum.app       start-up, shut-down
Q_DECLARE_LOGGING_CATEGORY(lcUi)       // vacuum.ui        what the user does, status bar messages
Q_DECLARE_LOGGING_CATEGORY(lcSim)      // vacuum.sim       RPC calls and commands
Q_DECLARE_LOGGING_CATEGORY(lcState)    // vacuum.state     state stream: events and a periodic summary
Q_DECLARE_LOGGING_CATEGORY(lcPose)     // vacuum.pose      pose estimator
Q_DECLARE_LOGGING_CATEGORY(lcMap)      // vacuum.map       cost map
Q_DECLARE_LOGGING_CATEGORY(lcPlan)     // vacuum.plan      path planner
Q_DECLARE_LOGGING_CATEGORY(lcNav)      // vacuum.nav       navigator decisions
Q_DECLARE_LOGGING_CATEGORY(lcNavTick)  // vacuum.nav.tick  control loop, a few lines per second
Q_DECLARE_LOGGING_CATEGORY(lcRec)      // vacuum.record    camera recording

// Streams without quotes or automatic spaces, so messages read like sentences.
#define VLOG_D(category) qCDebug(category).noquote().nospace()
#define VLOG_I(category) qCInfo(category).noquote().nospace()
#define VLOG_W(category) qCWarning(category).noquote().nospace()
#define VLOG_E(category) qCCritical(category).noquote().nospace()

namespace vacuum {

// Sends all log output to stderr and to `<directory>/vacuum_gui_<date>_<time>.log`.
// Returns the log file path, or an empty string if the file could not be created.
QString installLogHandler(const QString& directory);

// "(x, y, yaw°)" and "(x, y)", metres with millimetre precision.
QString str(const Pose2D& pose);
QString str(const Point2D& point);

QDebug operator<<(QDebug debug, const Pose2D& pose);
QDebug operator<<(QDebug debug, const Point2D& point);

// Rate limiter for log lines inside fast loops: ready() is true at most once per interval.
class LogThrottle {
 public:
  explicit LogThrottle(int intervalMs) : intervalMs_(intervalMs) {}
  bool ready() {
    if (timer_.isValid() && timer_.elapsed() < intervalMs_) return false;
    timer_.restart();
    return true;
  }

 private:
  int intervalMs_;
  QElapsedTimer timer_;
};

}  // namespace vacuum
