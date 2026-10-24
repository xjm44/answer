#include "MapView.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>

#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QImage>
#include <QMouseEvent>
#include <QPixmap>
#include <QWheelEvent>
#include <QtMath>

namespace {

constexpr double kTrailMinStep = 0.01;     // metres between stored trail points
constexpr double kGoalTolerance = 0.05;    // goal acceptance radius (assignment brief)
constexpr double kGoalArrowLength = 0.30;  // metres
constexpr double kTargetRadius = 0.09;
constexpr double kRawPoseRadius = 0.012;
constexpr double kMinPixelsPerMetre = 5.0;
constexpr double kMaxPixelsPerMetre = 3000.0;
constexpr int kPickDragPixels = 10;  // a right drag shorter than this is a click: no yaw

const QColor kBackground(0x2b, 0x2f, 0x36);
const QColor kFreeCell(0xfa, 0xfa, 0xfa);
const QColor kOccupiedCell(0x2d, 0x30, 0x38);
const QColor kGoalColor(0x1f, 0x9d, 0x55);
const QColor kSelectedGoalColor(0xff, 0x8c, 0x00);
const QColor kTrailColor(0x1e, 0x78, 0xff, 200);
const QColor kPathColor(0xd0, 0x2f, 0xd0);
const QColor kTargetColor(0xd0, 0x2f, 0xd0);
const QColor kRobotFill(0x3b, 0x82, 0xf6, 170);
const QColor kRobotBumpFill(0xef, 0x44, 0x44, 200);
const QColor kRawPoseColor(0xe5, 0x39, 0x35, 170);
const QColor kLearnedObstacle(0xe5, 0x39, 0x35);

// Pens are cosmetic: their width is in screen pixels whatever the zoom.
QPen cosmeticPen(const QColor& color, double width, Qt::PenStyle style = Qt::SolidLine) {
  QPen pen(color, width, style, Qt::RoundCap, Qt::RoundJoin);
  pen.setCosmetic(true);
  return pen;
}

// Arrow from (0, 0) along +x, for headings.
QPainterPath arrowPath(double length) {
  QPainterPath arrow;
  arrow.moveTo(0, 0);
  arrow.lineTo(length, 0);
  arrow.moveTo(length - 0.07, -0.04);
  arrow.lineTo(length, 0);
  arrow.lineTo(length - 0.07, 0.04);
  return arrow;
}

}  // namespace

MapView::MapView(QWidget* parent) : QGraphicsView(parent), scene_(new QGraphicsScene(this)) {
  setScene(scene_);
  setRenderHint(QPainter::Antialiasing);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setViewportUpdateMode(QGraphicsView::FullViewportUpdate);  // the scale bar overlay is in view coordinates
  setBackgroundBrush(kBackground);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);  // pan by dragging instead
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  viewport()->setMouseTracking(true);

  mapItem_ = scene_->addPixmap(QPixmap());
  mapItem_->setTransformationMode(Qt::FastTransformation);  // crisp cells when zoomed in
  mapItem_->setZValue(0);

  costItem_ = scene_->addPixmap(QPixmap());
  costItem_->setTransformationMode(Qt::FastTransformation);
  costItem_->setZValue(0.5);

  trailItem_ = scene_->addPath(QPainterPath(), cosmeticPen(kTrailColor, 2));
  trailItem_->setZValue(1);

  plannedPathItem_ = scene_->addPath(QPainterPath(), cosmeticPen(kPathColor, 2, Qt::DashLine));
  plannedPathItem_->setZValue(2);

  targetItem_ = scene_->addPath(QPainterPath(), cosmeticPen(kTargetColor, 2));
  targetItem_->setZValue(7);
  targetItem_->setVisible(false);

  pickItem_ = scene_->addPath(QPainterPath(), cosmeticPen(kTargetColor, 2, Qt::DashLine));
  pickItem_->setZValue(12);
  pickItem_->setVisible(false);

  robotItem_ = scene_->addEllipse(QRectF(), cosmeticPen(Qt::black, 2), kRobotFill);
  robotHeading_ = new QGraphicsLineItem(robotItem_);
  robotHeading_->setPen(cosmeticPen(Qt::black, 2));
  robotItem_->setZValue(10);
  robotItem_->setVisible(false);
  setRobotRadius(robotRadius_);

  rawPoseItem_ = scene_->addEllipse(QRectF(-kRawPoseRadius, -kRawPoseRadius, 2 * kRawPoseRadius, 2 * kRawPoseRadius),
                                    QPen(Qt::NoPen), kRawPoseColor);
  rawPoseItem_->setZValue(11);
  rawPoseItem_->setVisible(false);
  rawPoseItem_->setToolTip(tr("Raw (noisy) pose from the state stream"));
}

void MapView::placeOnMap(QGraphicsPixmapItem* item, const vacuum::MapMeta& meta) const {
  // Image pixel (c, r) -> world: scale by resolution, top-left corner at (ox, oy + H * res).
  item->setScale(meta.resolution);
  item->setPos(toScene(meta.origin.x, meta.origin.y + meta.height * meta.resolution));
}

void MapView::setMap(const vacuum::OccupancyMap& map) {
  if (!map.isValid()) {
    mapItem_->setPixmap(QPixmap());
    return;
  }
  const int w = map.width();
  const int h = map.height();

  QImage image(w, h, QImage::Format_Indexed8);
  QVector<QRgb> colors(256);
  for (int v = 0; v < 256; ++v)
    colors[v] = (v >= vacuum::OccupancyMap::kOccupiedThreshold ? kOccupiedCell : kFreeCell).rgb();
  image.setColorTable(colors);
  for (int row = 0; row < h; ++row)
    std::memcpy(image.scanLine(row), &map.cells()[static_cast<std::size_t>(row) * static_cast<std::size_t>(w)],
                static_cast<std::size_t>(w));

  mapItem_->setPixmap(QPixmap::fromImage(image));
  placeOnMap(mapItem_, map.meta());

  const QRectF bounds = mapItem_->sceneBoundingRect();
  scene_->setSceneRect(bounds.adjusted(-1.0, -1.0, 1.0, 1.0));  // a little room to pan past the edges
  autoFit_ = true;
  fitToMap();
}

void MapView::setCostMap(const vacuum::CostMap& costMap) {
  for (auto* item : learnedItems_) delete item;
  learnedItems_.clear();
  if (!costMap.isValid()) {
    costItem_->setPixmap(QPixmap());
    return;
  }

  // Red: the robot centre cannot go there. Orange, fading: allowed but costly (close to obstacles).
  const int w = costMap.width();
  const int h = costMap.height();
  const double weight = costMap.config().clearanceWeight;
  const QRgb blocked = qPremultiply(qRgba(229, 57, 53, 95));
  QImage image(w, h, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  for (int r = 0; r < h; ++r) {
    auto* line = reinterpret_cast<QRgb*>(image.scanLine(r));
    for (int c = 0; c < w; ++c) {
      if (costMap.isOccupied(c, r)) continue;
      if (!costMap.isTraversable(c, r)) {
        line[c] = blocked;
      } else {
        const double t = (costMap.costFactor(c, r) - 1.0) / weight;  // 0..1
        if (t > 0.02) line[c] = qPremultiply(qRgba(255, 160, 0, static_cast<int>(70 * t)));
      }
    }
  }
  costItem_->setPixmap(QPixmap::fromImage(image));
  placeOnMap(costItem_, costMap.meta());

  for (const auto& obstacle : costMap.learnedObstacles()) {
    const double r = obstacle.radius;
    QColor fill = kLearnedObstacle;
    fill.setAlpha(110);
    auto* item = scene_->addEllipse(QRectF(-r, -r, 2 * r, 2 * r), cosmeticPen(kLearnedObstacle, 2, Qt::DashLine), fill);
    item->setPos(toScene(obstacle.centre.x, obstacle.centre.y));
    item->setZValue(4);
    item->setToolTip(tr("Obstacle learned from a bumper hit (not on the map)"));
    learnedItems_.push_back(item);
  }
}

void MapView::setGoals(const std::vector<vacuum::Goal>& goals) {
  for (auto& entry : goals_) delete entry.second.body;  // children are deleted with their parent
  goals_.clear();

  for (const auto& goal : goals) {
    GoalMarker marker;
    marker.body = scene_->addEllipse(QRectF(-kGoalTolerance, -kGoalTolerance, 2 * kGoalTolerance, 2 * kGoalTolerance));
    marker.body->setPos(toScene(goal.x, goal.y));
    marker.body->setZValue(5);

    marker.arrow = new QGraphicsPathItem(marker.body);
    if (goal.yaw) {
      marker.arrow->setPath(arrowPath(kGoalArrowLength));
      marker.arrow->setRotation(-qRadiansToDegrees(*goal.yaw));  // scene y is flipped
    }

    const QString name = QString::fromStdString(goal.name);
    marker.label = new QGraphicsSimpleTextItem(name, marker.body);
    marker.label->setFlag(QGraphicsItem::ItemIgnoresTransformations);  // fixed pixel size
    marker.label->setTransform(QTransform::fromTranslate(8, -20));     // pixels, up-right of the marker

    marker.body->setToolTip(goal.yaw ? QStringLiteral("%1\n(%2, %3) m, %4°")
                                           .arg(name)
                                           .arg(goal.x, 0, 'f', 2)
                                           .arg(goal.y, 0, 'f', 2)
                                           .arg(qRadiansToDegrees(*goal.yaw), 0, 'f', 1)
                                     : QStringLiteral("%1\n(%2, %3) m").arg(name).arg(goal.x, 0, 'f', 2).arg(goal.y, 0, 'f', 2));

    styleGoal(marker, name == selectedGoal_);
    goals_[name] = marker;
  }
}

void MapView::setSelectedGoal(const QString& name) {
  selectedGoal_ = name;
  for (auto& entry : goals_) styleGoal(entry.second, entry.first == name);
}

void MapView::styleGoal(GoalMarker& marker, bool selected) {
  const QColor color = selected ? kSelectedGoalColor : kGoalColor;
  QColor fill = color;
  fill.setAlpha(selected ? 140 : 90);
  marker.body->setPen(cosmeticPen(color, selected ? 3 : 2));
  marker.body->setBrush(fill);
  marker.body->setZValue(selected ? 6 : 5);
  marker.arrow->setPen(cosmeticPen(color, selected ? 3 : 2));
  marker.label->setBrush(selected ? kSelectedGoalColor.darker(130) : kGoalColor.darker(150));
  QFont font = marker.label->font();
  font.setBold(selected);
  marker.label->setFont(font);
}

void MapView::setRobotRadius(double radius) {
  robotRadius_ = radius;
  robotItem_->setRect(QRectF(-radius, -radius, 2 * radius, 2 * radius));
  robotHeading_->setLine(0, 0, radius, 0);
}

void MapView::setRobotPose(const vacuum::Pose2D& pose, bool bumper) {
  const QPointF p = toScene(pose.x, pose.y);
  robotItem_->setPos(p);
  robotItem_->setRotation(-qRadiansToDegrees(pose.yaw));  // CCW world yaw, flipped scene y
  robotItem_->setBrush(bumper ? kRobotBumpFill : kRobotFill);
  robotItem_->setPen(cosmeticPen(bumper ? QColor(Qt::red) : QColor(Qt::black), 2));
  robotItem_->setVisible(true);

  appendTrailPoint(p);
  if (followRobot_) centerOn(p);
}

void MapView::setRawPose(const vacuum::Pose2D& pose) {
  rawPoseItem_->setPos(toScene(pose.x, pose.y));
  rawPoseItem_->setVisible(rawPoseVisible_);
}

void MapView::setPlannedPath(const std::vector<vacuum::Point2D>& path) {
  QPainterPath painterPath;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const QPointF p = toScene(path[i].x, path[i].y);
    if (i == 0)
      painterPath.moveTo(p);
    else
      painterPath.lineTo(p);
  }
  // Waypoint dots make the corners of the plan visible.
  for (std::size_t i = 1; i + 1 < path.size(); ++i) painterPath.addEllipse(toScene(path[i].x, path[i].y), 0.02, 0.02);
  plannedPathItem_->setPath(painterPath);
}

void MapView::setNavTarget(const std::optional<vacuum::Goal>& target) {
  if (!target) {
    targetItem_->setVisible(false);
    return;
  }
  QPainterPath path;
  path.addEllipse(QPointF(0, 0), kTargetRadius, kTargetRadius);
  path.moveTo(-1.5 * kTargetRadius, 0);
  path.lineTo(1.5 * kTargetRadius, 0);
  path.moveTo(0, -1.5 * kTargetRadius);
  path.lineTo(0, 1.5 * kTargetRadius);
  if (target->yaw) {
    QTransform rotate;
    rotate.rotate(-qRadiansToDegrees(*target->yaw));
    path.addPath(rotate.map(arrowPath(kGoalArrowLength)));
  }
  targetItem_->setPath(path);
  targetItem_->setPos(toScene(target->x, target->y));
  targetItem_->setToolTip(tr("Navigation target %1").arg(QString::fromStdString(target->name)));
  targetItem_->setVisible(true);
}

void MapView::appendTrailPoint(const QPointF& p) {
  if (trailEmpty_) {
    trail_.moveTo(p);
    trailEmpty_ = false;
  } else {
    if (QLineF(lastTrailPoint_, p).length() < kTrailMinStep) return;
    trail_.lineTo(p);
  }
  lastTrailPoint_ = p;
  trailItem_->setPath(trail_);
}

void MapView::clearTrail() {
  trail_ = QPainterPath();
  trailEmpty_ = true;
  trailItem_->setPath(trail_);
}

void MapView::setTrailVisible(bool visible) { trailItem_->setVisible(visible); }

void MapView::setRawPoseVisible(bool visible) {
  rawPoseVisible_ = visible;
  if (!visible) rawPoseItem_->setVisible(false);
}

void MapView::setCostOverlayVisible(bool visible) { costItem_->setVisible(visible); }

void MapView::setFollowRobot(bool follow) {
  followRobot_ = follow;
  if (follow) {
    autoFit_ = false;
    if (robotItem_->isVisible()) centerOn(robotItem_);
  }
}

void MapView::fitToMap() {
  const QRectF bounds = mapItem_->sceneBoundingRect();
  if (bounds.isEmpty()) return;
  autoFit_ = true;
  fitInView(bounds, Qt::KeepAspectRatio);
}

void MapView::wheelEvent(QWheelEvent* event) {
  const double current = transform().m11();  // pixels per metre
  double factor = std::pow(1.0015, event->angleDelta().y());
  factor = std::clamp(current * factor, kMinPixelsPerMetre, kMaxPixelsPerMetre) / current;
  scale(factor, factor);
  autoFit_ = false;
  event->accept();
}

void MapView::clearPickedPose() { pickItem_->setVisible(false); }

std::optional<double> MapView::pickedYaw(const QPoint& viewPos) const {
  if ((viewPos - pickViewPos_).manhattanLength() < kPickDragPixels) return std::nullopt;
  const QPointF d = mapToScene(viewPos) - pickScenePos_;
  return std::atan2(-d.y(), d.x());  // scene y is world -y
}

void MapView::updatePickMarker(const std::optional<double>& yaw) {
  QPainterPath path;
  path.addEllipse(QPointF(0, 0), kTargetRadius, kTargetRadius);
  if (yaw) {
    QTransform rotate;
    rotate.rotate(-qRadiansToDegrees(*yaw));
    path.addPath(rotate.map(arrowPath(kGoalArrowLength)));
  }
  pickItem_->setPath(path);
  pickItem_->setPos(pickScenePos_);
  pickItem_->setVisible(true);
}

void MapView::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::RightButton) {
    picking_ = true;
    pickViewPos_ = event->pos();
    pickScenePos_ = mapToScene(event->pos());
    updatePickMarker(std::nullopt);
    event->accept();
    return;
  }
  autoFit_ = false;  // the user is panning
  QGraphicsView::mousePressEvent(event);
}

void MapView::mouseMoveEvent(QMouseEvent* event) {
  const QPointF s = mapToScene(event->pos());
  emit cursorWorldPositionChanged(s.x(), -s.y());
  if (picking_) {
    updatePickMarker(pickedYaw(event->pos()));
    event->accept();
    return;
  }
  QGraphicsView::mouseMoveEvent(event);
}

void MapView::mouseReleaseEvent(QMouseEvent* event) {
  if (picking_ && event->button() == Qt::RightButton) {
    picking_ = false;
    const std::optional<double> yaw = pickedYaw(event->pos());
    updatePickMarker(yaw);  // stays on the map until Go to pose or the next pick
    emit worldPosePicked(pickScenePos_.x(), -pickScenePos_.y(), yaw.has_value(), yaw.value_or(0.0));
    event->accept();
    return;
  }
  QGraphicsView::mouseReleaseEvent(event);
}

void MapView::resizeEvent(QResizeEvent* event) {
  QGraphicsView::resizeEvent(event);
  if (autoFit_) fitToMap();
}

void MapView::drawForeground(QPainter* painter, const QRectF& rect) {
  QGraphicsView::drawForeground(painter, rect);

  // Scale bar in the bottom-left corner, drawn in viewport pixels.
  const double pixelsPerMetre = transform().m11();
  if (pixelsPerMetre <= 0.0) return;
  static const double kSteps[] = {0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0};
  double length = kSteps[std::size(kSteps) - 1];
  for (double step : kSteps) {
    if (step * pixelsPerMetre >= 60.0) {
      length = step;
      break;
    }
  }
  const double px = length * pixelsPerMetre;
  const QString text = length < 1.0 ? QStringLiteral("%1 cm").arg(qRound(length * 100))
                                    : QStringLiteral("%1 m").arg(length);

  painter->save();
  painter->resetTransform();
  const QPointF origin(16.0, viewport()->height() - 14.0);
  painter->setPen(Qt::NoPen);
  painter->setBrush(QColor(255, 255, 255, 200));
  painter->drawRoundedRect(QRectF(origin.x() - 8, origin.y() - 24, px + 16, 32), 4, 4);
  painter->setPen(QPen(Qt::black, 2));
  painter->drawLine(origin, origin + QPointF(px, 0));
  painter->drawLine(origin, origin + QPointF(0, -6));
  painter->drawLine(origin + QPointF(px, 0), origin + QPointF(px, -6));
  painter->drawText(QPointF(origin.x() + 4, origin.y() - 9), text);
  painter->restore();
}
