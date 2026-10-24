#include "core/recording/CameraRecorder.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <set>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>

#include "core/common/Log.h"
#include "core/sim/Protocol.h"
#include "core/sim/SimClient.h"
#include "core/sim/ZmqContext.h"

namespace vacuum {

namespace {

using protocol::Json;

constexpr int kPollTimeoutMs = 100;      // how quickly the thread notices a stop request
constexpr int kProgressIntervalMs = 500;
constexpr int kMaxMessagesPerPoll = 64;  // keep checking for a stop request under load
constexpr int kDepthPngCompression = 1;  // fastest zlib level; PNG stays lossless

QString utcNow() { return QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs); }

bool writeFile(const QString& path, const char* data, qint64 size) {
  QFile file(path);
  return file.open(QIODevice::WriteOnly) && file.write(data, size) == size;
}

// OpenCV takes narrow paths, which Windows reads in the local code page.
std::string cvPath(const QString& path) { return path.toLocal8Bit().toStdString(); }

// One video being filled frame by frame; opened on the first frame, when the image size is known.
struct VideoTrack {
  QString stem;  // "rgb" -> rgb.mp4, or rgb.avi if MPEG-4 is unavailable
  QString file;
  std::string codec;
  cv::VideoWriter writer;
  cv::Size size;
  int frames = 0;
  bool stopped = false;  // could not be opened: frames only go to frames/
};

// The files of one recording. Lives on the recorder thread.
class RecordingSession {
 public:
  RecordingSession(const QString& directory, RecordingSetup setup) : dir_(directory), setup_(std::move(setup)) {
    rgbVideo_.stem = QStringLiteral("rgb");
    depthVideo_.stem = QStringLiteral("depth_preview");
    const Json camera = setup_.cameraInfo.is_object() ? setup_.cameraInfo : Json::object();
    videoFps_ = camera.value("rate", 15.0);
    const double depthScale = camera.value("depth_scale", 0.001);
    maxDepthRaw_ = std::max(1.0, camera.value("max_depth", 6.0) / depthScale);
    for (const std::string& warning : setup_.warnings) warn(warning);
  }

  bool begin(QString* error) {
    if (!dir_.mkpath(QStringLiteral("frames/rgb")) || !dir_.mkpath(QStringLiteral("frames/depth"))) {
      *error = QStringLiteral("Cannot create %1").arg(dir_.absolutePath());
      return false;
    }
    framesCsv_.setFileName(dir_.filePath(QStringLiteral("frames.csv")));
    stateCsv_.setFileName(dir_.filePath(QStringLiteral("state.csv")));
    if (!framesCsv_.open(QIODevice::WriteOnly | QIODevice::Text) ||
        !stateCsv_.open(QIODevice::WriteOnly | QIODevice::Text)) {
      *error = QStringLiteral("Cannot write in %1").arg(dir_.absolutePath());
      return false;
    }
    framesCsv_.write("frame,seq,t,pose_x,pose_y,pose_yaw,recv_unix_ms,rgb_file,depth_file\n");
    stateCsv_.write("t,seq,scene,x,y,yaw,left_ticks,right_ticks,bumper,goto_id,goto_status,recv_unix_ms\n");

    if (!setup_.mapPng.isEmpty() &&
        !writeFile(dir_.filePath(QStringLiteral("map.png")), setup_.mapPng.constData(), setup_.mapPng.size()))
      warn("could not write map.png");

    startedUtc_ = utcNow();
    clock_.start();
    writeMetadata(false);  // a recording cut short still describes itself
    return true;
  }

  void addCameraFrame(const std::vector<zmq::message_t>& frames) {
    const Json header = frames.empty() ? Json() : Json::parse(frames[0].to_string(), nullptr, false);
    if (frames.size() < 2 || header.is_discarded() || !header.is_object() ||
        header.value("rgb", std::string("jpeg")) != "jpeg") {
      ++stats_.badMessages;
      warnOnce("skipped unreadable camera messages");
      return;
    }
    const qint64 recvMs = QDateTime::currentMSecsSinceEpoch();
    const auto seq = header.value("seq", std::int64_t{-1});
    const double t = header.value("t", 0.0);
    const int width = header.value("width", 0);
    const int height = header.value("height", 0);
    Pose2D pose;
    if (header.contains("pose") && header["pose"].is_object()) pose = protocol::poseFromJson(header["pose"]);

    // seq goes back after a load/reset of the simulator; only forward jumps are losses.
    if (lastSeq_ >= 0 && seq > lastSeq_ + 1) stats_.droppedFrames += seq - lastSeq_ - 1;
    lastSeq_ = seq;

    const QString name = QStringLiteral("%1").arg(stats_.frames, 6, 10, QLatin1Char('0'));
    QString rgbFile = QStringLiteral("frames/rgb/%1.jpg").arg(name);
    QString depthFile = QStringLiteral("frames/depth/%1.png").arg(name);

    // RGB: stored byte for byte as the camera sent it, decoded only for the video.
    if (!writeFile(dir_.filePath(rgbFile), frames[1].data<char>(), static_cast<qint64>(frames[1].size()))) {
      rgbFile.clear();
      warnOnce("could not write RGB frames");
    }

    // Depth: little-endian uint16, as on every platform we build for. Wrapped without a copy.
    const bool hasDepth = frames.size() >= 3 && header.value("depth", std::string("u16")) == "u16" && width > 0 &&
                          height > 0 && frames[2].size() == static_cast<size_t>(width) * height * 2;
    if (!hasDepth) {
      depthFile.clear();
      ++stats_.framesWithoutDepth;
      warnOnce("some camera frames had no valid depth image");
    }

    try {
      const cv::Mat jpeg(1, static_cast<int>(frames[1].size()), CV_8UC1, const_cast<void*>(frames[1].data()));
      cv::Mat rgb = cv::imdecode(jpeg, cv::IMREAD_COLOR);
      if (rgb.empty()) warnOnce("some RGB JPEGs could not be decoded; the video shows black for them");

      cv::Mat depthPreview;
      if (hasDepth) {
        const cv::Mat depth(height, width, CV_16UC1, const_cast<void*>(frames[2].data()));
        if (!cv::imwrite(cvPath(dir_.filePath(depthFile)), depth, {cv::IMWRITE_PNG_COMPRESSION, kDepthPngCompression})) {
          depthFile.clear();
          warnOnce("could not write depth frames");
        }
        depthPreview = colorizeDepth(depth);
      }

      // Both videos get a frame for every row of frames.csv, black when there is nothing to show.
      const cv::Size size = !rgb.empty() ? rgb.size() : cv::Size(width, height);
      appendVideo(rgbVideo_, rgb, size);
      appendVideo(depthVideo_, depthPreview, size);
    } catch (const cv::Exception& e) {
      warnOnce(std::string("OpenCV: ") + e.what());
    }

    framesCsv_.write(QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9\n")
                         .arg(stats_.frames)
                         .arg(seq)
                         .arg(t, 0, 'f', 6)
                         .arg(pose.x, 0, 'f', 4)
                         .arg(pose.y, 0, 'f', 4)
                         .arg(pose.yaw, 0, 'f', 5)
                         .arg(recvMs)
                         .arg(rgbFile, depthFile)
                         .toUtf8());
    ++stats_.frames;
  }

  void addState(const zmq::message_t& frame) {
    const auto json = Json::parse(frame.to_string(), nullptr, false);
    if (json.is_discarded() || !json.is_object()) return;
    RobotState state;
    try {
      state = protocol::robotStateFromJson(json);
    } catch (const std::exception&) {
      return;
    }
    if (!state.scene.empty()) scenesSeen_.insert(state.scene);
    stateCsv_.write(QStringLiteral("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12\n")
                        .arg(state.t, 0, 'f', 6)
                        .arg(state.seq)
                        .arg(QString::fromStdString(state.scene))
                        .arg(state.pose.x, 0, 'f', 4)
                        .arg(state.pose.y, 0, 'f', 4)
                        .arg(state.pose.yaw, 0, 'f', 5)
                        .arg(state.leftTicks)
                        .arg(state.rightTicks)
                        .arg(state.bumper ? 1 : 0)
                        .arg(state.gotoId ? QString::number(*state.gotoId) : QString())
                        .arg(QString::fromLatin1(state.gotoId ? toString(state.gotoStatus) : ""))
                        .arg(QDateTime::currentMSecsSinceEpoch())
                        .toUtf8());
    ++stats_.stateMessages;
  }

  void flush() {
    framesCsv_.flush();
    stateCsv_.flush();
  }

  void finish() {
    VLOG_I(lcRec) << "finishing: " << stats_.frames << " frames, rgb video " << rgbVideo_.frames
                  << " frames, depth video " << depthVideo_.frames << " frames";
    rgbVideo_.writer.release();
    depthVideo_.writer.release();
    framesCsv_.close();
    stateCsv_.close();
    writeMetadata(true);
  }

  const RecordingStats& stats() {
    stats_.seconds = clock_.isValid() ? clock_.elapsed() / 1000.0 : 0.0;
    return stats_;
  }

  std::vector<std::string> takeNewWarnings() { return std::exchange(newWarnings_, {}); }

 private:
  // TURBO colour map over 0 .. max_depth (blue near, red far); invalid pixels black.
  cv::Mat colorizeDepth(const cv::Mat& depth) const {
    cv::Mat scaled;
    depth.convertTo(scaled, CV_8U, 255.0 / maxDepthRaw_);
    cv::Mat colored;
    cv::applyColorMap(scaled, colored, cv::COLORMAP_TURBO);
    colored.setTo(cv::Scalar::all(0), depth == 0);
    return colored;
  }

  // MPEG-4 (.mp4, needs OpenCV's ffmpeg plugin), else Motion-JPEG (.avi, built into OpenCV).
  bool openVideo(VideoTrack& track, const cv::Size& size) {
    struct Choice {
      const char* extension;
      int api;
      const char* fourcc;
    };
    for (const Choice& choice : {Choice{"mp4", cv::CAP_ANY, "mp4v"}, Choice{"avi", cv::CAP_OPENCV_MJPEG, "MJPG"}}) {
      const QString file = QStringLiteral("%1.%2").arg(track.stem, QLatin1String(choice.extension));
      const int fourcc = cv::VideoWriter::fourcc(choice.fourcc[0], choice.fourcc[1], choice.fourcc[2], choice.fourcc[3]);
      try {
        if (track.writer.open(cvPath(dir_.filePath(file)), choice.api, fourcc, videoFps_, size, true)) {
          VLOG_I(lcRec) << "video " << file << ": " << choice.fourcc << " " << size.width << " x " << size.height
                        << " at " << videoFps_ << " fps, backend " << track.writer.getBackendName().c_str();
          track.file = file;
          track.codec = choice.fourcc;
          track.size = size;
          return true;
        }
      } catch (const cv::Exception& e) {
        VLOG_W(lcRec) << "could not open " << file << " (" << choice.fourcc << "): " << e.what();
      }
      if (track.file.isEmpty()) VLOG_W(lcRec) << "could not open " << file << " with codec " << choice.fourcc;
    }
    return false;
  }

  void appendVideo(VideoTrack& track, const cv::Mat& image, const cv::Size& size) {
    if (track.stopped || size.width <= 0 || size.height <= 0) return;
    if (!track.writer.isOpened() && !openVideo(track, size)) {
      track.stopped = true;
      warn(QStringLiteral("could not create %1 video; frames/ has every frame").arg(track.stem).toStdString());
      return;
    }
    if (image.empty()) {
      track.writer.write(cv::Mat::zeros(track.size, CV_8UC3));
    } else if (image.size() != track.size) {
      cv::Mat resized;
      cv::resize(image, resized, track.size);
      track.writer.write(resized);
    } else {
      track.writer.write(image);
    }
    ++track.frames;
  }

  void warn(const std::string& message) {
    VLOG_W(lcRec) << message.c_str();
    warnings_.push_back(message);
    newWarnings_.push_back(message);
  }

  void warnOnce(const std::string& message) {
    if (std::find(warnings_.begin(), warnings_.end(), message) == warnings_.end()) warn(message);
  }

  static Json videoJson(const VideoTrack& track) {
    if (track.file.isEmpty()) return nullptr;
    return {{"file", track.file.toStdString()}, {"codec", track.codec}, {"frames", track.frames}};
  }

  void writeMetadata(bool complete) {
    Json meta;
    meta["format_version"] = 1;
    meta["status"] = complete ? "complete" : "recording";  // "recording" after the end: the app died
    meta["started_utc"] = startedUtc_.toStdString();
    if (complete) meta["stopped_utc"] = utcNow().toStdString();
    meta["simulator"] = {{"host", setup_.host},
                         {"scene", setup_.scene},
                         {"scenes_in_state_stream", std::vector<std::string>(scenesSeen_.begin(), scenesSeen_.end())}};
    meta["camera_info"] = setup_.cameraInfo;
    meta["robot_info"] = setup_.robotInfo;
    meta["goals"] = setup_.goals;
    meta["map"] = setup_.mapInfo;
    if (!setup_.mapPng.isEmpty()) meta["map"]["file"] = "map.png";

    meta["videos"] = {{"fps", videoFps_}, {"rgb", videoJson(rgbVideo_)}, {"depth_preview", videoJson(depthVideo_)}};
    meta["files"] = {
        {"rgb.mp4",
         "RGB video (MPEG-4; rgb.avi Motion-JPEG if MPEG-4 was unavailable). Video frame i is row i of frames.csv. "
         "It plays at the nominal camera rate; frames.csv has the exact capture times."},
        {"depth_preview.mp4",
         "Colour-coded depth for viewing only (TURBO map over 0..max_depth: blue near, red far, black invalid). "
         "Frame i matches RGB video frame i. Use frames/depth for data."},
        {"frames/rgb/NNNNNN.jpg", "RGB frame NNNNNN of frames.csv, byte for byte as sent by the camera."},
        {"frames/depth/NNNNNN.png",
         "Depth frame NNNNNN: 16-bit grey PNG, metres = value * camera_info.depth_scale, 0 = invalid. Z-depth "
         "along the optical axis, pixel-aligned with RGB."},
        {"frames.csv",
         "One row per camera frame: frame index, camera seq, t = simulation time at capture (s), robot pose at "
         "capture (x, y in m, yaw in rad, world frame), recv_unix_ms = host receive time, file names."},
        {"state.csv", "Robot state stream (50 Hz) during the recording, on the same clock t."},
        {"map.png", "Occupancy map from get_map (0 free, 255 occupied); geometry in map."}};
    meta["notes"] = {
        "t restarts at 0 on load_scene / reset; the scene column of state.csv shows which scene was running.",
        "Camera extrinsics: camera_info.T_base_camera maps camera-frame points (OpenCV convention) to the robot "
        "frame; the robot pose maps the robot frame to the world."};

    const RecordingStats& s = stats();
    meta["stats"] = {{"frames", s.frames},
                     {"dropped_frames", s.droppedFrames},
                     {"frames_without_depth", s.framesWithoutDepth},
                     {"bad_messages", s.badMessages},
                     {"state_messages", s.stateMessages},
                     {"duration_s", s.seconds}};
    meta["warnings"] = warnings_;

    const std::string text = meta.dump(2);
    if (!writeFile(dir_.filePath(QStringLiteral("metadata.json")), text.data(), static_cast<qint64>(text.size())))
      warnOnce("could not write metadata.json");
  }

  QDir dir_;
  RecordingSetup setup_;
  double videoFps_ = 15.0;
  double maxDepthRaw_ = 6000.0;

  QFile framesCsv_;
  QFile stateCsv_;
  VideoTrack rgbVideo_;
  VideoTrack depthVideo_;

  RecordingStats stats_;
  QElapsedTimer clock_;
  std::int64_t lastSeq_ = -1;
  std::set<std::string> scenesSeen_;
  QString startedUtc_;
  std::vector<std::string> warnings_;
  std::vector<std::string> newWarnings_;
};

zmq::socket_t subscribe(const std::string& endpoint) {
  zmq::socket_t socket(sharedZmqContext(), zmq::socket_type::sub);
  socket.set(zmq::sockopt::linger, 0);
  socket.set(zmq::sockopt::subscribe, "");
  socket.connect(endpoint);
  return socket;
}

}  // namespace

RecordingSetup CameraRecorder::querySimulator(SimClient& client) {
  RecordingSetup setup;
  setup.host = client.host();
  auto query = [&](const char* method, auto&& store) {
    try {
      store();
    } catch (const std::exception& e) {
      setup.warnings.push_back(std::string(method) + " failed: " + e.what());
    }
  };
  query("get_camera_info", [&] { setup.cameraInfo = client.getCameraInfo(); });
  query("get_robot_info", [&] { setup.robotInfo = client.call("get_robot_info"); });
  query("get_goals", [&] { setup.goals = client.call("get_goals"); });
  query("get_map", [&] {
    std::vector<std::string> blobs;
    setup.mapInfo = client.call("get_map", Json::object(), &blobs);
    if (!blobs.empty()) setup.mapPng = QByteArray::fromStdString(blobs.front());
  });
  return setup;
}

QString CameraRecorder::createSessionDirectory(const QString& root, const QString& scene) {
  QString base = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd_HH-mm-ss"));
  if (!scene.isEmpty()) base += QStringLiteral("_scene") + scene;
  const QDir rootDir(root);
  QString name = base;
  for (int n = 2; rootDir.exists(name); ++n) name = QStringLiteral("%1_%2").arg(base).arg(n);
  if (!rootDir.mkpath(name)) return QString();
  return QDir::cleanPath(rootDir.absoluteFilePath(name));
}

CameraRecorder::CameraRecorder(QString directory, RecordingSetup setup, QObject* parent)
    : QThread(parent), directory_(std::move(directory)), setup_(std::move(setup)) {
  qRegisterMetaType<vacuum::RecordingStats>("vacuum::RecordingStats");
  setObjectName(QStringLiteral("recorder"));
}

CameraRecorder::~CameraRecorder() {
  requestInterruption();
  wait();
}

void CameraRecorder::run() {
  VLOG_I(lcRec) << "recording to " << directory_ << " (camera " << SimClient::endpoint(setup_.host, ports::kCamera).c_str()
                << ", state " << SimClient::endpoint(setup_.host, ports::kState).c_str() << ")";
  RecordingSession session(directory_, setup_);
  QString error;
  if (!session.begin(&error)) {
    VLOG_E(lcRec) << error;
    emit recordingError(error);
    return;
  }
  auto reportWarnings = [&] {
    for (const std::string& warning : session.takeNewWarnings()) emit recordingError(QString::fromStdString(warning));
  };
  reportWarnings();

  try {
    zmq::socket_t camera = subscribe(SimClient::endpoint(setup_.host, ports::kCamera));
    zmq::socket_t state = subscribe(SimClient::endpoint(setup_.host, ports::kState));
    std::vector<zmq::pollitem_t> items = {{camera.handle(), 0, ZMQ_POLLIN, 0}, {state.handle(), 0, ZMQ_POLLIN, 0}};

    QElapsedTimer sinceProgress;
    sinceProgress.start();
    LogThrottle summaryLog(5000);
    while (!isInterruptionRequested()) {
      zmq::poll(items, std::chrono::milliseconds(kPollTimeoutMs));

      // State first: it is small, and a burst of camera frames must not delay it.
      for (int i = 0; i < kMaxMessagesPerPoll * 4 && (items[1].revents & ZMQ_POLLIN); ++i) {
        zmq::message_t message;
        if (!state.recv(message, zmq::recv_flags::dontwait)) break;
        // The state stream is a single frame; drop any unexpected extra frames.
        while (message.more()) {
          zmq::message_t extra;
          if (!state.recv(extra, zmq::recv_flags::dontwait)) break;
        }
        session.addState(message);
      }
      for (int i = 0; i < kMaxMessagesPerPoll && (items[0].revents & ZMQ_POLLIN); ++i) {
        std::vector<zmq::message_t> frames;
        if (!zmq::recv_multipart(camera, std::back_inserter(frames), zmq::recv_flags::dontwait)) break;
        session.addCameraFrame(frames);
      }

      if (sinceProgress.elapsed() >= kProgressIntervalMs) {
        sinceProgress.restart();
        session.flush();
        reportWarnings();
        const RecordingStats& stats = session.stats();
        if (summaryLog.ready())
          VLOG_D(lcRec) << QString::number(stats.seconds, 'f', 1) << " s: " << stats.frames << " frames ("
                        << QString::number(stats.seconds > 0.0 ? stats.frames / stats.seconds : 0.0, 'f', 1)
                        << " fps), " << stats.droppedFrames << " dropped, " << stats.framesWithoutDepth
                        << " without depth, " << stats.stateMessages << " state messages";
        emit progress(stats);
      }
    }
  } catch (const zmq::error_t& e) {
    VLOG_E(lcRec) << "stream error: " << e.what();
    emit recordingError(QString::fromUtf8(e.what()));
  }

  session.finish();
  reportWarnings();
  emit progress(session.stats());
}

}  // namespace vacuum
