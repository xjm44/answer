#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <QMainWindow>
#include <QSet>
#include <QString>

#include "core/common/Types.h"
#include "core/navigation/Navigator.h"
#include "core/navigation/PoseEstimator.h"
#include "core/recording/CameraRecorder.h"

namespace vacuum {
class OccupancyMap;
class SimClient;
class StateSubscriber;
}  // namespace vacuum

namespace Ui {
class MainWindow;
}

class MapView;
class QPushButton;

// Application shell: connects to the simulator, loads scenes, shows them with the live
// robot, and hands goals / manual commands to the Navigator.
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

 private slots:
  void connectToSimulator();
  void loadSelectedScene();
  void resetScene();
  void onStateReceived(const vacuum::RobotState& state);
  void onStreamActiveChanged(bool active);
  void onGoalSelectionChanged();
  void goToSelectedGoal();
  void goToManualTarget();
  void onNavigatorModeChanged(vacuum::Navigator::Mode mode, const QString& detail);
  void onGoalMarked(const QString& name, double simTime);
  void startRecording();
  void stopRecording();
  void onRecordingProgress(const vacuum::RecordingStats& stats);
  void onRecordingFinished();
  void openRecordingFolder();

 private:
  MapView* mapView() const;
  void setupNavigation();
  void setupRecording();
  void updateRecordingControls();
  QString recordingRoot() const;
  void bindDriveButton(QPushButton* button, double linearSign, double angularSign);
  void disconnectFromSimulator();
  void startStateStream(const QString& host);
  void syncWithSimulator();
  void applyScene(const QString& sceneId, const std::vector<vacuum::Goal>& goals, const vacuum::OccupancyMap& map,
                  const std::optional<vacuum::Pose2D>& start);
  void refreshGoalList();
  void selectSceneInCombo(const QString& sceneId);
  void setControlsEnabled(bool enabled);
  std::optional<int> selectedSeed() const;
  void showError(const QString& context, const std::exception& error);

  std::unique_ptr<Ui::MainWindow> ui_;

  // Declared before the subscriber so the stream thread is stopped first on destruction.
  std::unique_ptr<vacuum::SimClient> client_;
  std::unique_ptr<vacuum::StateSubscriber> subscriber_;
  std::unique_ptr<vacuum::CameraRecorder> recorder_;  // kept after it finished, until the next recording
  vacuum::RecordingStats recordingStats_;             // latest progress of recorder_

  vacuum::Navigator* navigator_;  // child QObject
  vacuum::PoseEstimator estimator_;

  vacuum::RobotInfo robotInfo_;
  std::vector<vacuum::Goal> goals_;
  QSet<QString> markedGoals_;
  QString displayedScene_;          // scene whose map is on screen; empty until known
  bool awaitingSceneSync_ = false;  // drop stale state messages from before a load/reset
  QString warnedScene_;             // scene change by someone else that we already reported
  double lastStateTime_ = -1.0;
};
