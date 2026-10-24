#pragma once

#include <memory>

#include <QWidget>

class MapView;

namespace Ui {
class SceneViewWidget;
}

// The 2D map view plus its view controls (fit, follow robot, trail, raw pose, no-go zone, cursor position).
class SceneViewWidget : public QWidget {
  Q_OBJECT

 public:
  explicit SceneViewWidget(QWidget* parent = nullptr);
  ~SceneViewWidget() override;

  MapView* mapView() const;

 private:
  std::unique_ptr<Ui::SceneViewWidget> ui_;
};

