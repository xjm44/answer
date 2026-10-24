#include "SceneViewWidget.h"

#include "ui_SceneViewWidget.h"

SceneViewWidget::SceneViewWidget(QWidget* parent) : QWidget(parent), ui_(std::make_unique<Ui::SceneViewWidget>()) {
  ui_->setupUi(this);

  MapView* view = ui_->mapView;
  connect(ui_->fitButton, &QPushButton::clicked, this, [this, view] {
    ui_->followCheck->setChecked(false);
    view->fitToMap();
  });
  connect(ui_->followCheck, &QCheckBox::toggled, view, &MapView::setFollowRobot);
  connect(ui_->trailCheck, &QCheckBox::toggled, view, &MapView::setTrailVisible);
  connect(ui_->rawPoseCheck, &QCheckBox::toggled, view, &MapView::setRawPoseVisible);
  connect(ui_->costOverlayCheck, &QCheckBox::toggled, view, &MapView::setCostOverlayVisible);
  connect(ui_->clearTrailButton, &QPushButton::clicked, view, &MapView::clearTrail);
  connect(view, &MapView::cursorWorldPositionChanged, this, [this](double x, double y) {
    ui_->cursorLabel->setText(tr("x %1 m   y %2 m").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
  });
}

SceneViewWidget::~SceneViewWidget() = default;

MapView* SceneViewWidget::mapView() const { return ui_->mapView; }
