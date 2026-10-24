# Robot Vacuum Control

C++17 / Qt application for the Eureka Robotics vacuum simulator.

## Layout

```
answer/
├── CMakeLists.txt, cmake/       build; finds or downloads the dependencies
└── src/
    ├── main.cpp
    ├── core/                    no widgets
    │   ├── common/              types, geometry, logging
    │   ├── sim/                 ZeroMQ: RPC 5555, commands 5556, state stream 5557
    │   ├── map/                 occupancy grid; cost map with no-go zone and learned obstacles
    │   ├── navigation/          Task 1: PoseEstimator, PathPlanner (A*), Navigator (20 Hz controller)
    │   └── recording/           Task 2: CameraRecorder (camera 5558 + state stream -> folder)
    └── ui/                      Qt Widgets, laid out in .ui files
        ├── MainWindow.*         connection, scenes, goals, manual driving, robot status, recording
        └── widgets/             Task 3: map view and its toolbar
```

## How the robot gets to a goal

1. **Pose estimate.** The streamed pose jitters by about 1 cm / 0.5° (std). `PoseEstimator` predicts motion from the wheel encoders and pulls it slightly towards each streamed pose, which averages out the noise.
2. **Cost map.** Every cell stores its distance to the nearest obstacle. Cells closer than robot radius + 2 cm are no-go. Below 35 cm the cost rises, so paths keep to the middle of free space.
3. **Planner.** A* on the grid, then shortened into a few straight segments that keep the same clearance. If there is no path with the 2 cm margin, it tries again with 0.5 cm. A goal in the no-go zone is planned to the nearest safe cell (within 30 cm), then reached in a straight line.
4. **Navigator.** Sends wheel commands every 50 ms and follows the path with pure pursuit, slowing to 8 cm/s where clearance is under 28 cm. At the goal it approaches precisely, turns to the goal's yaw, stands still for 0.5 s, and calls `mark_goal` only if within 3 cm / 3°. The simulator allows 5 cm / 5°, and only the first mark counts.
5. **Collisions.** On a bump it works out where the contact is, adds an unmapped obstacle to the cost map, backs away and re-plans. It stops with **Failed** after 12 collisions, 13 escape moves for one contact, or 25 re-plans (`NavigatorConfig` in `Navigator.h`), and also when the state stream goes quiet. The planner does not check whether the robot fits at the target: a **Go to pose** target closer to a wall than the robot radius (17 cm) ends with *too many collisions*.

## Camera recording

**Start recording** / **Stop recording** (Tab 2) writes a new folder `screenRecording/<date>_<time>_scene<id>/`:

| File | Content |
|---|---|
| `rgb.mp4` | RGB video (Motion-JPEG `rgb.avi` if OpenCV cannot write MP4) |
| `depth_preview.mp4` | colour-coded depth, for viewing |
| `frames/rgb/*.jpg`, `frames/depth/*.png` | every frame; depth as lossless 16-bit PNG (metres = value × `depth_scale`, 0 = invalid) |
| `frames.csv` | one row per frame: `seq`, sim time `t`, robot pose, file names |
| `state.csv` | the 50 Hz state stream, same clock |
| `map.png`, `metadata.json` | the map; camera intrinsics and `T_base_camera`, robot info, goals, stats |

Row *i* of `frames.csv` is frame *i* of both videos.

## Logging

Logs go to the console and to `logs/vacuum_gui_<date>_<time>.log` in the working directory:

```
16:02:03.123 I [main] vacuum.nav: collision #1 at (-1.650, 1.150, 25.6°): ...
```

| Category | Content |
|---|---|
| `vacuum.app`, `vacuum.ui` | start-up, user actions, status bar messages |
| `vacuum.sim` | every RPC with request, reply and duration; commands |
| `vacuum.state` | stream health, bumper, 2 s summary with the raw pose |
| `vacuum.pose`, `vacuum.map`, `vacuum.plan` | estimator resets and outliers, learned obstacles, every plan |
| `vacuum.nav` | mode changes, collisions, re-plans, arrival checks, failures |
| `vacuum.nav.tick` | control loop, about 5 lines a second |
| `vacuum.record` | recording progress and summary |

To silence a category: `set QT_LOGGING_RULES=vacuum.nav.tick.debug=false`.

## Build

Needs Qt 5.15 or 6, OpenCV 4, ZeroMQ 4.3.5 + cppzmq 4.10 and nlohmann_json 3.11. OpenCV must be installed; CMake downloads the other three if they are missing (`-DVACUUM_FETCH_DEPS=OFF` turns this off).

Windows (Visual Studio 2022, Qt 5.15.2, OpenCV 4.10 `opencv_world`):

```powershell
cmake -S answer -B answer/build -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH=C:/Qt/5.15.2/msvc2019_64 `
      -DOpenCV_DIR=D:/Library/Builded/opencv_ALL/opencv_4.10.0/x64-release
cmake --build answer/build --config Release
```

The output is `answer/build/bin/Release/vacuum_gui.exe`, with the Qt, ZeroMQ and OpenCV DLLs copied next to it. For Debug builds, set `-DVACUUM_OPENCV_DEBUG_DIR=<path>` unless OpenCV's debug build is in a sibling `x64-debug` folder. An OpenCV built with CUDA also needs the CUDA DLLs on `PATH`.

## Run

```bash
docker run --rm -p 5555-5558:5555-5558 -p 8080:8080 ghcr.io/eurekarobotics/vacuum-sim:1.0.0 --scene 01 --viewer
vacuum_gui      # connects to localhost and shows the scene already loaded
```

- **Tab 1:** connect; load or reset a scene. Tick **Seed** for a repeatable start pose.
- **Goals:** select a goal and press **GO** (or double-click it). Marked goals get a ✔.
- **Manually GOTO:** hold ▲ ▼ ◀ ▶ to drive; **Stop** or **Esc** stops. **Go to pose** drives to x / y and an optional yaw. Right-click the map to pick x / y; right-drag to also set the yaw.
- **Map:** scroll to zoom, drag to pan. Blue disc: filtered pose. Red dot: raw pose. Red / orange overlay: no-go / avoided zone. Dashed red circles: learned obstacles. Magenta dashed line: planned path. The toolbar above the map shows or hides these.
- **Robot panel:** sim time, pose, wheel ticks, bumper, stream health, and the navigator's state with its reason.
