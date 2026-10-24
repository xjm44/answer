# Resolves ZeroMQ (libzmq + cppzmq) and nlohmann_json.
#
# Installed packages are preferred (e.g. apt/brew/vcpkg). If one is missing and
# VACUUM_FETCH_DEPS is ON, it is downloaded and built as part of this project.
#
# Outputs:
#   nlohmann_json::nlohmann_json   target
#   VACUUM_ZMQ_TARGET              name of a target providing <zmq.hpp> and libzmq

include(FetchContent)

option(VACUUM_FETCH_DEPS "Download ZeroMQ, cppzmq and nlohmann_json if they are not installed" ON)

# libzmq 4.3.5 and nlohmann_json 3.11 declare cmake_minimum_required() < 3.5, which CMake 4 rejects.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)  # extracted files get the extraction time as timestamp
endif()

set(VACUUM_ZMQ_SHARED_TARGET "")  # set when libzmq is built here, so its DLL can be copied next to the app

# --- nlohmann_json ----------------------------------------------------------
find_package(nlohmann_json 3.2 CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
  if(NOT VACUUM_FETCH_DEPS)
    message(FATAL_ERROR "nlohmann_json not found and VACUUM_FETCH_DEPS is OFF")
  endif()
  message(STATUS "nlohmann_json not found: downloading v3.11.3")
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

# --- ZeroMQ -----------------------------------------------------------------
find_package(cppzmq CONFIG QUIET)
if(cppzmq_FOUND)
  set(VACUUM_ZMQ_TARGET cppzmq)
else()
  if(NOT VACUUM_FETCH_DEPS)
    message(FATAL_ERROR "cppzmq not found and VACUUM_FETCH_DEPS is OFF")
  endif()
  message(STATUS "cppzmq not found: downloading libzmq v4.3.5 and cppzmq v4.10.0")

  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(ZMQ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED ON CACHE BOOL "" FORCE)
  set(BUILD_STATIC OFF CACHE BOOL "" FORCE)
  set(WITH_PERF_TOOL OFF CACHE BOOL "" FORCE)
  set(WITH_DOCS OFF CACHE BOOL "" FORCE)
  set(ENABLE_CPACK OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(libzmq
    URL https://github.com/zeromq/libzmq/releases/download/v4.3.5/zeromq-4.3.5.tar.gz)

  # cppzmq is header-only; point SOURCE_SUBDIR at a folder without a CMakeLists.txt so
  # its own build (tests, install/export rules) is skipped and we only get the headers.
  FetchContent_Declare(cppzmq
    URL https://github.com/zeromq/cppzmq/archive/refs/tags/v4.10.0.tar.gz
    SOURCE_SUBDIR headers-only)

  FetchContent_MakeAvailable(libzmq cppzmq)

  add_library(vacuum_cppzmq INTERFACE)
  target_include_directories(vacuum_cppzmq INTERFACE ${cppzmq_SOURCE_DIR} ${libzmq_SOURCE_DIR}/include)
  target_link_libraries(vacuum_cppzmq INTERFACE libzmq)
  set(VACUUM_ZMQ_TARGET vacuum_cppzmq)
  set(VACUUM_ZMQ_SHARED_TARGET libzmq)
endif()

# --- OpenCV -----------------------------------------------------------------
# Camera recording: video encoding, PNG / JPEG codecs, depth colour map.
# Point CMake at the install, e.g. -DOpenCV_DIR=D:/Library/Builded/opencv_ALL/opencv_4.10.0/x64-release
#
# Outputs:
#   VACUUM_OPENCV_TARGETS          OpenCV targets to link
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio)
message(STATUS "OpenCV ${OpenCV_VERSION}: ${OpenCV_DIR}")

if(TARGET opencv_world)
  set(VACUUM_OPENCV_TARGETS opencv_world)

  # Release and debug Windows builds often live in separate installs (.../x64-release, .../x64-debug).
  # Without the debug DLL, Debug builds would mix the debug and release C++ runtimes.
  get_target_property(_vacuum_cv_configs opencv_world IMPORTED_CONFIGURATIONS)
  if(MSVC AND NOT "DEBUG" IN_LIST _vacuum_cv_configs)
    string(REPLACE "x64-release" "x64-debug" _vacuum_cv_debug_guess "${OpenCV_DIR}")
    if(_vacuum_cv_debug_guess STREQUAL OpenCV_DIR)
      set(_vacuum_cv_debug_guess "")
    endif()
    set(VACUUM_OPENCV_DEBUG_DIR "${_vacuum_cv_debug_guess}" CACHE PATH
        "OpenCV install with the debug opencv_world, used for Debug builds")
    if(VACUUM_OPENCV_DEBUG_DIR)
      file(GLOB_RECURSE _vacuum_cv_debug_lib "${VACUUM_OPENCV_DEBUG_DIR}/opencv_world*d.lib")
      file(GLOB_RECURSE _vacuum_cv_debug_dll "${VACUUM_OPENCV_DEBUG_DIR}/opencv_world*d.dll")
    endif()
    if(_vacuum_cv_debug_lib AND _vacuum_cv_debug_dll)
      list(GET _vacuum_cv_debug_lib 0 _vacuum_cv_debug_lib)
      list(GET _vacuum_cv_debug_dll 0 _vacuum_cv_debug_dll)
      set_property(TARGET opencv_world APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(opencv_world PROPERTIES
        IMPORTED_IMPLIB_DEBUG "${_vacuum_cv_debug_lib}"
        IMPORTED_LOCATION_DEBUG "${_vacuum_cv_debug_dll}")
      message(STATUS "OpenCV debug: ${_vacuum_cv_debug_dll}")
    else()
      message(WARNING "No debug opencv_world found (set VACUUM_OPENCV_DEBUG_DIR): Debug builds use the release DLL")
    endif()
  endif()
else()
  set(VACUUM_OPENCV_TARGETS ${OpenCV_LIBS})
endif()
