cmake_minimum_required(VERSION 3.16)
project(vacuum_control VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep executables and DLLs (including a fetched libzmq) in one folder so the app runs from the build tree.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
# Qt: pass -DCMAKE_PREFIX_PATH=<Qt>/<kit>, e.g. C:/Qt/5.15.2/msvc2019_64
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

# ZeroMQ (libzmq + cppzmq) and nlohmann_json: installed packages, or downloaded. OpenCV: installed.
include(cmake/Dependencies.cmake)

# Enabled after the dependencies so Qt's code generators only run on our own targets.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

function(vacuum_set_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  if(WIN32)
    # zmq.h pulls in <winsock2.h>; keep windows.h from defining min/max.
    target_compile_definitions(${target} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
  endif()
endfunction()

# ---------------------------------------------------------------------------
# vacuum_core: simulator protocol, transport and map model (no widgets)
# ---------------------------------------------------------------------------
add_library(vacuum_core STATIC
  src/core/common/Types.h
  src/core/common/Types.cpp
  src/core/common/Geometry.h
  src/core/common/Log.h
  src/core/common/Log.cpp
  src/core/sim/Protocol.h
  src/core/sim/Protocol.cpp
  src/core/sim/ZmqContext.h
  src/core/sim/SimClient.h
  src/core/sim/SimClient.cpp
  src/core/sim/StateSubscriber.h
  src/core/sim/StateSubscriber.cpp
  src/core/map/OccupancyMap.h
  src/core/map/OccupancyMap.cpp
  src/core/map/CostMap.h
  src/core/map/CostMap.cpp
  src/core/navigation/PoseEstimator.h
  src/core/navigation/PoseEstimator.cpp
  src/core/navigation/PathPlanner.h
  src/core/navigation/PathPlanner.cpp
  src/core/navigation/Navigator.h
  src/core/navigation/Navigator.cpp
  src/core/recording/CameraRecorder.h
  src/core/recording/CameraRecorder.cpp
)
target_include_directories(vacuum_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(vacuum_core
  PUBLIC
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    nlohmann_json::nlohmann_json
  PRIVATE
    ${VACUUM_ZMQ_TARGET}
    ${VACUUM_OPENCV_TARGETS}
)
vacuum_set_warnings(vacuum_core)

# ---------------------------------------------------------------------------
# vacuum_gui: Qt Widgets application
# ---------------------------------------------------------------------------
add_executable(vacuum_gui
  src/main.cpp
  src/ui/MainWindow.h
  src/ui/MainWindow.cpp
  src/ui/MainWindow.ui
  src/ui/widgets/SceneViewWidget.h
  src/ui/widgets/SceneViewWidget.cpp
  src/ui/widgets/SceneViewWidget.ui
  src/ui/widgets/MapView.h
  src/ui/widgets/MapView.cpp
)
# uic-generated headers include promoted widgets by bare name (e.g. "MapView.h").
target_include_directories(vacuum_gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/widgets)
target_link_libraries(vacuum_gui PRIVATE vacuum_core Qt${QT_VERSION_MAJOR}::Widgets)
vacuum_set_warnings(vacuum_gui)

# On Windows, copy the runtime DLLs (libzmq, Qt) next to the executable so it starts from the build tree.
if(WIN32)
  if(VACUUM_ZMQ_SHARED_TARGET)
    add_custom_command(TARGET vacuum_gui POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "$<TARGET_FILE:${VACUUM_ZMQ_SHARED_TARGET}>" "$<TARGET_FILE_DIR:vacuum_gui>"
      VERBATIM)
  endif()

  # OpenCV DLLs, plus its ffmpeg plugin, which cv::VideoWriter loads at runtime.
  set(_vacuum_cv_plugins "")
  foreach(_cv_target IN LISTS VACUUM_OPENCV_TARGETS)
    get_target_property(_cv_type ${_cv_target} TYPE)
    if(NOT _cv_type STREQUAL "SHARED_LIBRARY")
      continue()
    endif()
    add_custom_command(TARGET vacuum_gui POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:${_cv_target}>" "$<TARGET_FILE_DIR:vacuum_gui>"
      VERBATIM)
    get_target_property(_cv_dll ${_cv_target} IMPORTED_LOCATION_RELEASE)
    if(_cv_dll)
      get_filename_component(_cv_bin "${_cv_dll}" DIRECTORY)
      file(GLOB _cv_found "${_cv_bin}/opencv_videoio_ffmpeg*.dll")
      list(APPEND _vacuum_cv_plugins ${_cv_found})
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _vacuum_cv_plugins)
  foreach(_cv_plugin IN LISTS _vacuum_cv_plugins)
    add_custom_command(TARGET vacuum_gui POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_cv_plugin}" "$<TARGET_FILE_DIR:vacuum_gui>"
      VERBATIM)
  endforeach()

  get_target_property(_vacuum_qmake Qt${QT_VERSION_MAJOR}::qmake IMPORTED_LOCATION)
  get_filename_component(_vacuum_qt_bin "${_vacuum_qmake}" DIRECTORY)
  find_program(VACUUM_WINDEPLOYQT windeployqt HINTS "${_vacuum_qt_bin}")
  if(VACUUM_WINDEPLOYQT)
    add_custom_command(TARGET vacuum_gui POST_BUILD
      COMMAND "${VACUUM_WINDEPLOYQT}" --no-translations --no-compiler-runtime "$<TARGET_FILE:vacuum_gui>"
      COMMENT "Deploying Qt runtime next to vacuum_gui"
      VERBATIM)
  endif()
endif()
