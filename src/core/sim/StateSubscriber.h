#pragma once

#include <QMetaType>
#include <QString>
#include <QThread>

#include "core/common/Types.h"

namespace vacuum {

// Receives the 50 Hz robot state stream (port 5557) on its own thread and
// re-emits every message as a Qt signal (delivered queued to receivers in other threads).
class StateSubscriber : public QThread {
  Q_OBJECT

 public:
  // `endpoint` e.g. "tcp://localhost:5557". A stream silent for `staleAfterMs` is reported inactive.
  explicit StateSubscriber(QString endpoint, int staleAfterMs = 1000, QObject* parent = nullptr);
  ~StateSubscriber() override;  // stops and joins the thread

 signals:
  void stateReceived(const vacuum::RobotState& state);
  void streamActiveChanged(bool active);
  void streamError(const QString& message);

 protected:
  void run() override;

 private:
  QString endpoint_;
  int staleAfterMs_;
};

}  // namespace vacuum

Q_DECLARE_METATYPE(vacuum::RobotState)
