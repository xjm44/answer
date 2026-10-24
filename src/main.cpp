#include <QApplication>
#include <QDir>

#include "core/common/Log.h"
#include "ui/MainWindow.h"

int main(int argc, char* argv[]) {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#endif
  QApplication app(argc, argv);
  QApplication::setApplicationName(QStringLiteral("vacuum_gui"));
  QApplication::setApplicationVersion(QStringLiteral("0.1.0"));

  const QString logFile = vacuum::installLogHandler(QDir::current().absoluteFilePath(QStringLiteral("logs")));
  VLOG_I(lcApp) << "vacuum_gui " << QApplication::applicationVersion() << ", Qt " << qVersion() << ", working directory "
                << QDir::currentPath();
  if (logFile.isEmpty()) VLOG_W(lcApp) << "could not create a log file: console output only";
  else VLOG_I(lcApp) << "log file " << logFile;

  MainWindow window;
  window.show();
  const int code = app.exec();
  VLOG_I(lcApp) << "exiting with code " << code;
  return code;
}
