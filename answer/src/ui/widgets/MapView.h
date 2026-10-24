#pragma once

#include <map>
#include <optional>
#include <vector>

#include <QGraphicsView>
#include <QPainterPath>
#include <QString>

#include "core/common/Types.h"
#include "core/map/CostMap.h"
#include "core/map/OccupancyMap.h"

class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QGraphicsPathItem;
class QGraphicsPixmapItem;
class QGraphicsSimpleTextItem;

// Top-down 2D view of a scene: occupancy map, planning overlay, goals, robot (filtered
// and raw pose), trail, planned path and navigation target.
//
// Scene coordinates are world metres with y flipped (scene = (x, -y)), so the
// world frame's +y points up on screen. Wheel zooms, left-drag pans, right-click picks a point,
// right-drag picks a point and the heading towards where the button is released.
class MapView : public QGraphicsView {
  Q_OBJECT

 public:
  explicit MapView(QWidget* parent = nullptr);

  void setMap(const vacuum::OccupancyMap& map);
  void setCostMap(const vacuum::CostMap& costMap);  // no-go overlay + learned obstacles
  void setGoals(const std::vector<vacuum::Goal>& goals);
  void setSelectedGoal(const QString& name);
  void setRobotRadius(double radius);
  void setRobotPose(const vacuum::Pose2D& pose, bool bumper);  // filtered estimate
  void setRawPose(const vacuum::Pose2D& pose);                  // noisy measurement
  void setPlannedPath(const std::vector<vacuum::Point2D>& path);
  void setNavTarget(const std::optional<vacuum::Goal>& target);
  void clearPickedPose();  // hides the marker of the last right-click / right-drag pick

 public slots:
  void clearTrail();
  void setTrailVisible(bool visible);
  void setRawPoseVisible(bool visible);
  void setCostOverlayVisible(bool visible);
  void setFollowRobot(bool follow);
  void fitToMap();

 signals:
  void cursorWorldPositionChanged(double x, double y);
  void worldPosePicked(double x, double y, bool hasYaw, double yaw);  // right click (no yaw) / right drag

 protected:
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void drawForeground(QPainter* painter, const QRectF& rect) override;

 private:
  struct GoalMarker {
    QGraphicsEllipseItem* body = nullptr;  // goal tolerance disc; parent of arrow and label
    QGraphicsPathItem* arrow = nullptr;    // required heading, if any
    QGraphicsSimpleTextItem* label = nullptr;
  };

  static QPointF toScene(double x, double y) { return {x, -y}; }
  void placeOnMap(QGraphicsPixmapItem* item, const vacuum::MapMeta& meta) const;
  void styleGoal(GoalMarker& marker, bool selected);
  void appendTrailPoint(const QPointF& p);
  std::optional<double> pickedYaw(const QPoint& viewPos) const;  // nullopt until dragged far enough
  void updatePickMarker(const std::optional<double>& yaw);

  QGraphicsScene* scene_;
  QGraphicsPixmapItem* mapItem_;
  QGraphicsPixmapItem* costItem_;
  QGraphicsPathItem* trailItem_;
  QGraphicsPathItem* plannedPathItem_;
  QGraphicsPathItem* targetItem_;
  QGraphicsPathItem* pickItem_;  // pose picked with the right button, shown until cleared
  QGraphicsEllipseItem* robotItem_;
  QGraphicsLineItem* robotHeading_;
  QGraphicsEllipseItem* rawPoseItem_;
  std::vector<QGraphicsEllipseItem*> learnedItems_;

  std::map<QString, GoalMarker> goals_;
  QString selectedGoal_;

  QPainterPath trail_;
  QPointF lastTrailPoint_;
  bool trailEmpty_ = true;

  double robotRadius_ = 0.17;
  bool rawPoseVisible_ = true;
  bool followRobot_ = false;
  bool autoFit_ = true;  // keep the map fitted on resize until the user zooms or pans
  bool picking_ = false;  // right button held
  QPoint pickViewPos_;
  QPointF pickScenePos_;
};
