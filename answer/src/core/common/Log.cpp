#include "core/common/Log.h"

#include <cstdio>
#include <cstdlib>
#include <mutex>

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QThread>

#include "core/common/Geometry.h"

Q_LOGGING_CATEGORY(lcApp, "vacuum.app")
Q_LOGGING_CATEGORY(lcUi, "vacuum.ui")
Q_LOGGING_CATEGORY(lcSim, "vacuum.sim")
Q_LOGGING_CATEGORY(lcState, "vacuum.state")
Q_LOGGING_CATEGORY(lcPose, "vacuum.pose")
Q_LOGGING_CATEGORY(lcMap, "vacuum.map")
Q_LOGGING_CATEGORY(lcPlan, "vacuum.plan")
Q_LOGGING_CATEGORY(lcNav, "vacuum.nav")
Q_LOGGING_CATEGORY(lcNavTick, "vacuum.nav.tick")
Q_LOGGING_CATEGORY(lcRec, "vacuum.record")

namespace vacuum {

namespace {

std::mutex gLogMutex;
QFile* gLogFile = nullptr;  // lives until the process ends, so late messages are still written

char levelLetter(QtMsgType type) {
  switch (type) {
    case QtDebugMsg: return 'D';
    case QtInfoMsg: return 'I';
    case QtWarningMsg: return 'W';
    case QtCriticalMsg: return 'E';
    case QtFatalMsg: return 'F';
  }
  return '?';
}

// "16:02:03.123 I [main] vacuum.nav: message"
void writeLog(QtMsgType type, const QMessageLogContext& context, const QString& message) {
  QThread* current = QThread::currentThread();
  QString thread = current ? current->objectName() : QString();
  if (thread.isEmpty()) {
    const QCoreApplication* app = QCoreApplication::instance();
    thread = app && current == app->thread() ? QStringLiteral("main") : QStringLiteral("worker");
  }
  const QByteArray line = QStringLiteral("%1 %2 [%3] %4: %5\n")
                              .arg(QTime::currentTime().toString(QStringLiteral("HH:mm:ss.zzz")))
                              .arg(QLatin1Char(levelLetter(type)))
                              .arg(thread, QString::fromLatin1(context.category ? context.category : "default"),
                                   message)
                              .toUtf8();
  {
    std::lock_guard<std::mutex> lock(gLogMutex);
    std::fwrite(line.constData(), 1, static_cast<size_t>(line.size()), stderr);
    std::fflush(stderr);
    if (gLogFile) {
      gLogFile->write(line);
      gLogFile->flush();  // a crash must not lose the lines that explain it
    }
  }
  if (type == QtFatalMsg) std::abort();
}

}  // namespace

QString installLogHandler(const QString& directory) {
  QString path;
  {
    std::lock_guard<std::mutex> lock(gLogMutex);
    if (QDir().mkpath(directory)) {
      path = QDir(directory).absoluteFilePath(
          QStringLiteral("vacuum_gui_%1.log").arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd_HH-mm-ss"))));
      auto* file = new QFile(path);
      if (file->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        gLogFile = file;
      } else {
        delete file;
        path.clear();
      }
    }
  }
  qInstallMessageHandler(writeLog);
  return path;
}

QString str(const Pose2D& pose) {
  return QStringLiteral("(%1, %2, %3°)")
      .arg(pose.x, 0, 'f', 3)
      .arg(pose.y, 0, 'f', 3)
      .arg(radToDeg(pose.yaw), 0, 'f', 1);
}

QString str(const Point2D& point) {
  return QStringLiteral("(%1, %2)").arg(point.x, 0, 'f', 3).arg(point.y, 0, 'f', 3);
}

QDebug operator<<(QDebug debug, const Pose2D& pose) {
  const QDebugStateSaver saver(debug);
  debug.noquote().nospace() << str(pose);
  return debug;
}

QDebug operator<<(QDebug debug, const Point2D& point) {
  const QDebugStateSaver saver(debug);
  debug.noquote().nospace() << str(point);
  return debug;
}

}  // namespace vacuum
