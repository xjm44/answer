#include "MainWindow.h"

#include <algorithm>

#include <QApplication>
#include <QDesktopServices>
#include <QDir>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QShortcut>
#include <QStatusBar>
#include <QTimer>
#include <QUrl>
#include <QtMath>

#include "MapView.h"
#include "SceneViewWidget.h"
#include "core/common/Log.h"
#include "core/map/OccupancyMap.h"
#include "core/sim/SimClient.h"
#include "core/sim/StateSubscriber.h"
#include "ui_MainWindow.h"

namespace {

// Shows a wait cursor while blocking RPCs (e.g. load_scene, ~1 s) run.
class BusyCursor {
 public:
  BusyCursor() { QApplication::setOverrideCursor(Qt::WaitCursor); }
  ~BusyCursor() { QApplication::restoreOverrideCursor(); }
  BusyCursor(const BusyCursor&) = delete;
  BusyCursor& operator=(const BusyCursor&) = delete;
};

constexpr int kGoalNameRole = Qt::UserRole;

QString goalText(const vacuum::Goal& goal, bool marked) {
  QString text = QStringLiteral("%1   (%2, %3)")
                     .arg(QString::fromStdString(goal.name))
                     .arg(goal.x, 0, 'f', 2)
                     .arg(goal.y, 0, 'f', 2);
  if (goal.yaw) text += QStringLiteral("  %1°").arg(qRadiansToDegrees(*goal.yaw), 0, 'f', 0);
  if (marked) text += QStringLiteral("   ✔ marked");
  return text;
}

// The goal the user picked. Not currentItem(): the list makes its first row current (without
// selecting it) when it gets keyboard focus, e.g. when its tab is opened.
QListWidgetItem* selectedGoalItem(const QListWidget* list) {
  const QList<QListWidgetItem*> items = list->selectedItems();
  return items.isEmpty() ? nullptr : items.first();
}

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(std::make_unique<Ui::MainWindow>()), navigator_(new vacuum::Navigator(this)) {
  ui_->setupUi(this);

  connect(ui_->connectButton, &QPushButton::clicked, this, &MainWindow::connectToSimulator);
  connect(ui_->hostEdit, &QLineEdit::returnPressed, this, &MainWindow::connectToSimulator);
  connect(ui_->loadSceneButton, &QPushButton::clicked, this, &MainWindow::loadSelectedScene);
  connect(ui_->resetButton, &QPushButton::clicked, this, &MainWindow::resetScene);
  connect(ui_->goalList, &QListWidget::itemSelectionChanged, this, &MainWindow::onGoalSelectionChanged);
  setupNavigation();
  setupRecording();
  connect(statusBar(), &QStatusBar::messageChanged, this, [](const QString& message) {
    if (!message.isEmpty()) VLOG_D(lcUi) << "status bar: " << message;
  });

  setControlsEnabled(false);
  QTimer::singleShot(0, this, &MainWindow::connectToSimulator);  // connect once the window is up
}

MainWindow::~MainWindow() {
  VLOG_I(lcUi) << "main window closing";
  disconnectFromSimulator();
}

MapView* MainWindow::mapView() const { return ui_->sceneView->mapView(); }

void MainWindow::setupNavigation() {
  // Goals
  ui_->gotoGoalButton->setEnabled(false);
  ui_->gotoGoalButton->setToolTip(tr("Plan a path to the selected goal, drive there and mark it (double-click a goal does the same)"));
  connect(ui_->gotoGoalButton, &QPushButton::clicked, this, &MainWindow::goToSelectedGoal);
  connect(ui_->goalList, &QListWidget::itemDoubleClicked, this, &MainWindow::goToSelectedGoal);

  // Manual driving: buttons drive while held; Stop / Esc stops everything.
  bindDriveButton(ui_->forwardButton, 1.0, 0.0);
  bindDriveButton(ui_->backwardButton, -1.0, 0.0);
  bindDriveButton(ui_->leftButton, 0.0, 1.0);
  bindDriveButton(ui_->rightButton, 0.0, -1.0);
  connect(ui_->stopButton, &QPushButton::clicked, this, [this] {
    VLOG_I(lcUi) << "Stop button";
    navigator_->stop();
  });
  auto* escape = new QShortcut(QKeySequence(Qt::Key_Escape), this);
  escape->setContext(Qt::ApplicationShortcut);
  connect(escape, &QShortcut::activated, this, [this] {
    VLOG_I(lcUi) << "Esc pressed: stop";
    navigator_->stop();
  });

  // Manual target
  connect(ui_->gotoPoseButton, &QPushButton::clicked, this, &MainWindow::goToManualTarget);
  connect(mapView(), &MapView::worldPosePicked, this, [this](double x, double y, bool hasYaw, double yaw) {
    VLOG_I(lcUi) << "map pick: (" << QString::number(x, 'f', 3) << ", " << QString::number(y, 'f', 3) << ")"
                 << (hasYaw ? QStringLiteral(" yaw %1°").arg(qRadiansToDegrees(yaw), 0, 'f', 1) : QString());
    ui_->targetXSpin->setValue(x);
    ui_->targetYSpin->setValue(y);
    if (hasYaw) {  // a plain click keeps the yaw settings as they are
      ui_->useTargetYawCheck->setChecked(true);
      ui_->targetYawSpin->setValue(qRadiansToDegrees(yaw));
    }
    const QString target = QStringLiteral("(%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2);
    statusBar()->showMessage(
        hasYaw ? tr("Target set to %1, yaw %2° - press Go to pose").arg(target).arg(qRadiansToDegrees(yaw), 0, 'f', 1)
               : tr("Target set to %1 - press Go to pose").arg(target),
        4000);
  });

  // Navigator -> view / status
  connect(navigator_, &vacuum::Navigator::modeChanged, this, &MainWindow::onNavigatorModeChanged);
  connect(navigator_, &vacuum::Navigator::pathChanged, this,
          [this](const std::vector<vacuum::Point2D>& path) { mapView()->setPlannedPath(path); });
  connect(navigator_, &vacuum::Navigator::costMapChanged, this,
          [this] { mapView()->setCostMap(navigator_->costMap()); });
  connect(navigator_, &vacuum::Navigator::goalMarked, this, &MainWindow::onGoalMarked);
}

void MainWindow::bindDriveButton(QPushButton* button, double linearSign, double angularSign) {
  connect(button, &QPushButton::pressed, this, [this, button, linearSign, angularSign] {
    VLOG_I(lcUi) << "drive button '" << button->objectName() << "' pressed";
    navigator_->driveManual(linearSign * ui_->linearSpeedSpin->value(), angularSign * ui_->angularSpeedSpin->value());
  });
  connect(button, &QPushButton::released, this, [this, button] {
    VLOG_I(lcUi) << "drive button '" << button->objectName() << "' released";
    navigator_->driveManual(0.0, 0.0);
  });
}

void MainWindow::disconnectFromSimulator() {
  navigator_->stop();
  navigator_->setClient(nullptr);
  if (recorder_) {  // finish the files; onRecordingFinished() reports it
    recorder_->requestInterruption();
    recorder_->wait();
  }
  subscriber_.reset();  // joins the stream thread
  client_.reset();
}

void MainWindow::connectToSimulator() {
  disconnectFromSimulator();
  setControlsEnabled(false);
  displayedScene_.clear();
  warnedScene_.clear();
  lastStateTime_ = -1.0;

  const QString host = ui_->hostEdit->text().trimmed();
  VLOG_I(lcUi) << "connecting to the simulator at " << host;
  ui_->connectionLabel->setText(tr("Connecting to %1 ...").arg(host));

  std::vector<vacuum::SceneInfo> scenes;
  try {
    BusyCursor busy;
    auto client = std::make_unique<vacuum::SimClient>(host.toStdString());
    client->ping();
    scenes = client->listScenes();
    robotInfo_ = client->getRobotInfo();
    client_ = std::move(client);
  } catch (const std::exception& e) {
    ui_->connectionLabel->setText(tr("Not connected"));
    showError(tr("Could not connect to the simulator at %1").arg(host), e);
    return;
  }

  ui_->connectionLabel->setText(tr("Connected to %1").arg(host));
  VLOG_I(lcUi) << "connected to " << host << ": " << scenes.size() << " scenes; robot radius " << robotInfo_.radius
               << " m, wheel base " << robotInfo_.wheelBase << " m, wheel radius " << robotInfo_.wheelRadius
               << " m, max " << robotInfo_.maxLinearSpeed << " m/s / " << robotInfo_.maxAngularSpeed
               << " rad/s, watchdog " << robotInfo_.watchdog << " s";
  ui_->sceneCombo->clear();
  for (const auto& scene : scenes) {
    const QString id = QString::fromStdString(scene.id);
    ui_->sceneCombo->addItem(QStringLiteral("%1 - %2 (%3)")
                                 .arg(id, QString::fromStdString(scene.name), QString::fromStdString(scene.difficulty)),
                             id);
  }
  mapView()->setRobotRadius(robotInfo_.radius);
  estimator_.setRobotInfo(robotInfo_);
  navigator_->setRobotInfo(robotInfo_);
  navigator_->setClient(client_.get());
  setControlsEnabled(true);
  startStateStream(host);
  syncWithSimulator();
}

void MainWindow::startStateStream(const QString& host) {
  subscriber_ = std::make_unique<vacuum::StateSubscriber>(
      QString::fromStdString(vacuum::SimClient::endpoint(host.toStdString(), vacuum::ports::kState)));
  connect(subscriber_.get(), &vacuum::StateSubscriber::stateReceived, this, &MainWindow::onStateReceived);
  connect(subscriber_.get(), &vacuum::StateSubscriber::streamActiveChanged, this,
          &MainWindow::onStreamActiveChanged);
  connect(subscriber_.get(), &vacuum::StateSubscriber::streamError, this,
          [this](const QString& message) { statusBar()->showMessage(tr("State stream error: %1").arg(message)); });
  ui_->streamValue->setText(tr("waiting ..."));
  subscriber_->start();
}

// Shows the scene that is already loaded in the simulator, without resetting it.
// Its id is taken from the first state message.
void MainWindow::syncWithSimulator() {
  if (!client_) return;
  VLOG_I(lcUi) << "showing the scene already loaded in the simulator";
  try {
    BusyCursor busy;
    const auto goals = client_->getGoals();
    const auto map = client_->getMap();
    applyScene(QString(), goals, map, std::nullopt);
    statusBar()->showMessage(tr("Showing the scene currently loaded in the simulator"), 5000);
  } catch (const std::exception& e) {
    showError(tr("Could not read the current scene"), e);
  }
}

void MainWindow::loadSelectedScene() {
  if (!client_) return;
  const QString id = ui_->sceneCombo->currentData().toString();
  if (id.isEmpty()) return;
  VLOG_I(lcUi) << "Load scene " << id << (selectedSeed() ? QStringLiteral(" seed %1").arg(*selectedSeed()) : QString());
  navigator_->stop();
  try {
    BusyCursor busy;
    const auto result = client_->loadScene(id.toStdString(), selectedSeed());
    const auto map = client_->getMap();
    applyScene(id, result.goals, map, result.start);
    statusBar()->showMessage(tr("Loaded scene %1, start (%2, %3) m")
                                 .arg(id)
                                 .arg(result.start.x, 0, 'f', 2)
                                 .arg(result.start.y, 0, 'f', 2),
                             5000);
  } catch (const std::exception& e) {
    showError(tr("Could not load scene %1").arg(id), e);
  }
}

void MainWindow::resetScene() {
  if (!client_) return;
  VLOG_I(lcUi) << "Reset scene" << (selectedSeed() ? QStringLiteral(" seed %1").arg(*selectedSeed()) : QString());
  navigator_->stop();
  try {
    BusyCursor busy;
    const auto result = client_->reset(selectedSeed());
    const auto map = client_->getMap();
    const QString id = result.id.empty() ? displayedScene_ : QString::fromStdString(result.id);
    applyScene(id, result.goals, map, result.start);
    statusBar()->showMessage(tr("Reset scene %1").arg(id), 5000);
  } catch (const std::exception& e) {
    showError(tr("Could not reset the scene"), e);
  }
}

void MainWindow::applyScene(const QString& sceneId, const std::vector<vacuum::Goal>& goals,
                            const vacuum::OccupancyMap& map, const std::optional<vacuum::Pose2D>& start) {
  VLOG_I(lcUi) << "scene '" << sceneId << "': map " << map.width() << " x " << map.height() << " cells, "
               << goals.size() << " goals, start "
               << (start ? vacuum::str(*start) : QStringLiteral("unknown (taken from the state stream)"));
  for (const auto& goal : goals)
    VLOG_D(lcUi) << "  goal '" << goal.name.c_str() << "' at (" << goal.x << ", " << goal.y << ")"
                 << (goal.yaw ? QStringLiteral(" yaw %1°").arg(qRadiansToDegrees(*goal.yaw), 0, 'f', 1) : QString());
  goals_ = goals;
  markedGoals_.clear();
  displayedScene_ = sceneId;
  awaitingSceneSync_ = !sceneId.isEmpty();
  warnedScene_.clear();
  lastStateTime_ = -1.0;

  // load_scene reports the exact start pose; otherwise estimate it from the stream.
  if (start)
    estimator_.reset(*start);
  else
    estimator_.reset();

  MapView* view = mapView();
  view->setMap(map);
  navigator_->setMap(map);  // emits costMapChanged -> overlay
  view->setGoals(goals_);
  view->setPlannedPath({});
  view->setNavTarget(std::nullopt);
  view->clearTrail();
  if (start) view->setRobotPose(*start, false);

  refreshGoalList();
  view->setSelectedGoal(QString());
  ui_->gotoGoalButton->setEnabled(false);

  if (!sceneId.isEmpty()) selectSceneInCombo(sceneId);
  setWindowTitle(sceneId.isEmpty() ? tr("Vacuum Control") : tr("Vacuum Control - scene %1").arg(sceneId));
}

void MainWindow::refreshGoalList() {
  const QListWidgetItem* selected = selectedGoalItem(ui_->goalList);
  const QString current = selected ? selected->data(kGoalNameRole).toString() : QString();
  const QSignalBlocker blocker(ui_->goalList);
  ui_->goalList->clear();
  for (const auto& goal : goals_) {
    const QString name = QString::fromStdString(goal.name);
    auto* item = new QListWidgetItem(goalText(goal, markedGoals_.contains(name)), ui_->goalList);
    item->setData(kGoalNameRole, name);
    if (markedGoals_.contains(name)) item->setForeground(QColor(0x1f, 0x9d, 0x55));
    if (name == current) ui_->goalList->setCurrentItem(item);
  }
}

void MainWindow::onStateReceived(const vacuum::RobotState& state) {
  const QString scene = QString::fromStdString(state.scene);

  if (awaitingSceneSync_) {
    if (scene != displayedScene_) return;  // queued from before our load/reset
    awaitingSceneSync_ = false;
  }
  if (displayedScene_.isEmpty() && !scene.isEmpty()) {  // first message after syncWithSimulator()
    VLOG_I(lcUi) << "the simulator is running scene " << scene;
    displayedScene_ = scene;
    selectSceneInCombo(scene);
    setWindowTitle(tr("Vacuum Control - scene %1").arg(scene));
  } else if (!scene.isEmpty() && scene != displayedScene_ && scene != warnedScene_) {
    warnedScene_ = scene;
    VLOG_W(lcUi) << "the simulator switched to scene " << scene << " (showing " << displayedScene_
                 << "): stopping, the map is wrong now";
    navigator_->stop();  // our map is wrong now
    statusBar()->showMessage(
        tr("The simulator switched to scene %1 - press Load scene to refresh the map").arg(scene));
  }

  // Simulation time restarts on load/reset; a jump back means the old trail is meaningless.
  if (lastStateTime_ >= 0.0 && state.t + 0.5 < lastStateTime_) {
    VLOG_I(lcUi) << "sim time went back " << lastStateTime_ << " -> " << state.t << " s: trail cleared";
    mapView()->clearTrail();
  }
  lastStateTime_ = state.t;

  const vacuum::Pose2D estimate = estimator_.update(state);
  navigator_->onState(state, estimate);

  mapView()->setRobotPose(estimate, state.bumper);
  mapView()->setRawPose(state.pose);

  ui_->timeValue->setText(tr("%1 s").arg(state.t, 0, 'f', 2));
  ui_->poseValue->setText(tr("x %1   y %2   %3°")
                              .arg(estimate.x, 0, 'f', 3)
                              .arg(estimate.y, 0, 'f', 3)
                              .arg(qRadiansToDegrees(estimate.yaw), 0, 'f', 1));
  ui_->ticksValue->setText(tr("L %1   R %2").arg(state.leftTicks).arg(state.rightTicks));
  ui_->bumperValue->setText(state.bumper ? tr("<b><font color='#d32f2f'>CONTACT</font></b>") : tr("clear"));
}

void MainWindow::onStreamActiveChanged(bool active) {
  ui_->streamValue->setText(active ? tr("receiving") : tr("<font color='#d32f2f'>no data</font>"));
}

void MainWindow::onGoalSelectionChanged() {
  const QListWidgetItem* item = selectedGoalItem(ui_->goalList);
  mapView()->setSelectedGoal(item ? item->data(kGoalNameRole).toString() : QString());
  ui_->gotoGoalButton->setEnabled(item != nullptr);
}

void MainWindow::goToSelectedGoal() {
  const QListWidgetItem* item = selectedGoalItem(ui_->goalList);
  if (!item) {
    statusBar()->showMessage(tr("Select a goal first"), 3000);
    return;
  }
  const QString name = item->data(kGoalNameRole).toString();
  const auto it = std::find_if(goals_.begin(), goals_.end(),
                               [&](const vacuum::Goal& g) { return QString::fromStdString(g.name) == name; });
  if (it == goals_.end()) return;
  VLOG_I(lcUi) << "GO to goal '" << name << "'" << (markedGoals_.contains(name) ? " (already marked)" : "");

  vacuum::NavTarget target;
  target.x = it->x;
  target.y = it->y;
  target.yaw = it->yaw;
  target.goalName = it->name;
  mapView()->setNavTarget(*it);
  navigator_->navigateTo(target);
}

void MainWindow::goToManualTarget() {
  vacuum::Goal shown;
  shown.name = "manual target";
  shown.x = ui_->targetXSpin->value();
  shown.y = ui_->targetYSpin->value();
  if (ui_->useTargetYawCheck->isChecked()) shown.yaw = qDegreesToRadians(ui_->targetYawSpin->value());
  VLOG_I(lcUi) << "Go to pose (" << shown.x << ", " << shown.y << ")"
               << (shown.yaw ? QStringLiteral(" yaw %1°").arg(ui_->targetYawSpin->value(), 0, 'f', 1)
                             : QStringLiteral(" no yaw"));

  vacuum::NavTarget target;
  target.x = shown.x;
  target.y = shown.y;
  target.yaw = shown.yaw;  // no goal name: nothing to mark
  mapView()->clearPickedPose();
  mapView()->setNavTarget(shown);
  navigator_->navigateTo(target);
}

void MainWindow::onNavigatorModeChanged(vacuum::Navigator::Mode mode, const QString& detail) {
  using Mode = vacuum::Navigator::Mode;
  QString color = QStringLiteral("#1565c0");
  if (mode == Mode::Arrived) color = QStringLiteral("#2e7d32");
  if (mode == Mode::Failed || mode == Mode::Recovering) color = QStringLiteral("#d32f2f");
  if (mode == Mode::Idle) color = QStringLiteral("#555555");

  const QString name = QString::fromLatin1(vacuum::Navigator::toString(mode));
  ui_->navValue->setText(detail.isEmpty() ? QStringLiteral("<b><font color='%1'>%2</font></b>").arg(color, name)
                                          : QStringLiteral("<b><font color='%1'>%2</font></b><br>%3")
                                                .arg(color, name, detail.toHtmlEscaped()));
  if (!detail.isEmpty()) statusBar()->showMessage(QStringLiteral("%1: %2").arg(name, detail), 8000);
  if (mode == Mode::Idle || mode == Mode::Arrived || mode == Mode::Failed || mode == Mode::Manual)
    mapView()->setNavTarget(std::nullopt);
}

void MainWindow::onGoalMarked(const QString& name, double simTime) {
  VLOG_I(lcUi) << "goal '" << name << "' marked at t = " << simTime << " s";
  markedGoals_.insert(name);
  refreshGoalList();
  statusBar()->showMessage(tr("Goal '%1' marked at t = %2 s").arg(name).arg(simTime, 0, 'f', 2), 8000);
}

void MainWindow::setupRecording() {
  connect(ui_->startRecordingButton, &QPushButton::clicked, this, &MainWindow::startRecording);
  connect(ui_->stopRecordingButton, &QPushButton::clicked, this, &MainWindow::stopRecording);
  connect(ui_->openRecordingFolderButton, &QPushButton::clicked, this, &MainWindow::openRecordingFolder);
  ui_->openRecordingFolderButton->setToolTip(QDir::toNativeSeparators(recordingRoot()));
  updateRecordingControls();
}

QString MainWindow::recordingRoot() const { return QDir::current().absoluteFilePath(QStringLiteral("screenRecording")); }

void MainWindow::updateRecordingControls() {
  const bool recording = recorder_ && recorder_->isRunning();
  ui_->startRecordingButton->setEnabled(client_ != nullptr && !recording);
  ui_->stopRecordingButton->setEnabled(recording && !recorder_->isInterruptionRequested());
}

void MainWindow::startRecording() {
  if (!client_ || (recorder_ && recorder_->isRunning())) return;
  VLOG_I(lcUi) << "Start recording";

  vacuum::RecordingSetup setup;
  {
    BusyCursor busy;
    setup = vacuum::CameraRecorder::querySimulator(*client_);
  }
  setup.scene = displayedScene_.toStdString();

  const QString directory = vacuum::CameraRecorder::createSessionDirectory(recordingRoot(), displayedScene_);
  if (directory.isEmpty()) {
    QMessageBox::warning(this, tr("Vacuum Control"),
                         tr("Could not create a recording folder in %1").arg(QDir::toNativeSeparators(recordingRoot())));
    return;
  }

  recorder_ = std::make_unique<vacuum::CameraRecorder>(directory, std::move(setup));
  connect(recorder_.get(), &vacuum::CameraRecorder::progress, this, &MainWindow::onRecordingProgress);
  connect(recorder_.get(), &vacuum::CameraRecorder::recordingError, this, [this](const QString& message) {
    VLOG_W(lcRec) << message;
    statusBar()->showMessage(tr("Recording: %1").arg(message), 10000);
  });
  connect(recorder_.get(), &QThread::finished, this, &MainWindow::onRecordingFinished);
  recordingStats_ = {};
  recorder_->start();

  onRecordingProgress(recordingStats_);
  updateRecordingControls();
}

void MainWindow::stopRecording() {
  if (!recorder_ || !recorder_->isRunning()) return;
  VLOG_I(lcUi) << "Stop recording";
  recorder_->requestInterruption();  // the recorder thread finishes the files, then onRecordingFinished()
  ui_->recordingStatusLabel->setText(tr("Finishing the recording ..."));
  updateRecordingControls();
}

void MainWindow::onRecordingProgress(const vacuum::RecordingStats& stats) {
  recordingStats_ = stats;
  if (!recorder_ || !recorder_->isRunning() || recorder_->isInterruptionRequested()) return;

  const int seconds = static_cast<int>(stats.seconds);
  ui_->recordingStatusLabel->setText(
      tr("<b><font color='#d32f2f'>● REC %1:%2</font></b>   %3 frames, %4 dropped<br>%5")
          .arg(seconds / 60, 2, 10, QLatin1Char('0'))
          .arg(seconds % 60, 2, 10, QLatin1Char('0'))
          .arg(stats.frames)
          .arg(stats.droppedFrames)
          .arg(QDir::toNativeSeparators(recorder_->directory()).toHtmlEscaped()));
}

void MainWindow::onRecordingFinished() {
  if (!recorder_ || recorder_->isRunning()) return;
  const vacuum::RecordingStats& stats = recordingStats_;
  const QString directory = QDir::toNativeSeparators(recorder_->directory());
  QString text = tr("Saved %1 frames (%2 s, %3 dropped) to<br>%4")
                     .arg(stats.frames)
                     .arg(stats.seconds, 0, 'f', 1)
                     .arg(stats.droppedFrames)
                     .arg(directory.toHtmlEscaped());
  if (stats.frames == 0)
    text += tr("<br><font color='#d32f2f'>No camera frames arrived - is port %1 reachable?</font>")
                .arg(vacuum::ports::kCamera);
  ui_->recordingStatusLabel->setText(text);
  VLOG_I(lcRec) << "recording finished: " << stats.frames << " frames, " << stats.droppedFrames << " dropped, "
                << stats.framesWithoutDepth << " without depth, " << stats.stateMessages << " state messages, "
                << QString::number(stats.seconds, 'f', 1) << " s -> " << directory;
  statusBar()->showMessage(tr("Camera recording saved to %1").arg(directory), 8000);
  updateRecordingControls();
}

void MainWindow::openRecordingFolder() {
  const QString root = recordingRoot();
  QDir().mkpath(root);
  QDesktopServices::openUrl(QUrl::fromLocalFile(root));
}

void MainWindow::selectSceneInCombo(const QString& sceneId) {
  const int index = ui_->sceneCombo->findData(sceneId);
  if (index >= 0) ui_->sceneCombo->setCurrentIndex(index);
}

void MainWindow::setControlsEnabled(bool enabled) {
  ui_->sceneGroup->setEnabled(enabled);
  ui_->goalsGroup->setEnabled(enabled);
  ui_->manualGOTOGroup->setEnabled(enabled);
  updateRecordingControls();
}

std::optional<int> MainWindow::selectedSeed() const {
  if (!ui_->useSeedCheck->isChecked()) return std::nullopt;
  return ui_->seedSpin->value();
}

void MainWindow::showError(const QString& context, const std::exception& error) {
  const QString message = QStringLiteral("%1:\n%2").arg(context, QString::fromUtf8(error.what()));
  VLOG_W(lcUi) << "error dialog: " << message.simplified();
  statusBar()->showMessage(message.simplified(), 10000);
  QMessageBox::warning(this, tr("Vacuum Control"), message);
}
