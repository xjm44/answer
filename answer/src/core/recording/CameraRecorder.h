#pragma once

#include <string>
#include <vector>

#include <QByteArray>
#include <QMetaType>
#include <QString>
#include <QThread>

#include <nlohmann/json.hpp>

namespace vacuum {

class SimClient;

// What the simulator reports once, saved with the recording so that it is self-contained.
struct RecordingSetup {
  std::string host;
  std::string scene;           // scene on screen when recording started; may be empty
  nlohmann::json cameraInfo;   // get_camera_info: intrinsics, depth_scale, T_base_camera ...
  nlohmann::json robotInfo;    // get_robot_info
  nlohmann::json goals;        // get_goals
  nlohmann::json mapInfo;      // get_map header
  QByteArray mapPng;           // get_map image
  std::vector<std::string> warnings;  // what could not be queried
};

struct RecordingStats {
  qint64 frames = 0;              // camera frames saved
  qint64 droppedFrames = 0;       // gaps in the camera seq counter
  qint64 framesWithoutDepth = 0;  // depth payload missing or malformed
  qint64 badMessages = 0;         // unreadable camera messages, not saved
  qint64 stateMessages = 0;
  double seconds = 0.0;           // wall time since the start
};

// Records the RGBD camera stream (port 5558) and the robot state stream (port 5557) on its own
// thread into one directory:
//
//   rgb.mp4              RGB video (OpenCV VideoWriter; rgb.avi Motion-JPEG if MPEG-4 is unavailable)
//   depth_preview.mp4    colour-coded depth, for viewing
//   frames/rgb/*.jpg     every RGB frame as received
//   frames/depth/*.png   every depth frame, 16-bit, lossless
//   frames.csv           per frame: seq, sim time, robot pose at capture, receive time, files
//   state.csv            robot state stream (50 Hz) on the same clock
//   map.png              occupancy map
//   metadata.json        camera calibration, robot info, goals, map header, file formats, stats
//
// Stop with requestInterruption(); the files are finalised before the thread ends.
class CameraRecorder : public QThread {
  Q_OBJECT

 public:
  // Queries what a recording needs from the simulator. Never throws: failures go to `warnings`.
  static RecordingSetup querySimulator(SimClient& client);

  // Creates a new, empty directory `<root>/<date>_<time>[_scene<id>]`. Returns an empty string on failure.
  static QString createSessionDirectory(const QString& root, const QString& scene);

  CameraRecorder(QString directory, RecordingSetup setup, QObject* parent = nullptr);
  ~CameraRecorder() override;  // stops, finalises the files and joins the thread

  const QString& directory() const { return directory_; }

 signals:
  void progress(const vacuum::RecordingStats& stats);  // about twice a second, and once at the end
  void recordingError(const QString& message);

 protected:
  void run() override;

 private:
  QString directory_;
  RecordingSetup setup_;
};

}  // namespace vacuum

Q_DECLARE_METATYPE(vacuum::RecordingStats)
