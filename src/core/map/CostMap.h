#pragma once

#include <cstdint>
#include <vector>

#include "core/common/Types.h"
#include "core/map/OccupancyMap.h"

namespace vacuum {

struct CostMapConfig {
  double robotRadius = 0.17;
  double safetyMargin = 0.02;        // hard clearance on top of the robot radius
  double preferredClearance = 0.35;  // closer than this to an obstacle costs extra
  double clearanceWeight = 8.0;      // cost multiplier at the hard limit is 1 + weight
};

// Obstacle that is not on the map but was discovered at run time (bumper hit).
struct LearnedObstacle {
  Point2D centre;
  double radius = 0.0;
};

// Planning view of the occupancy map: the map plus learned obstacles, with the
// clearance (distance to the nearest obstacle) of every cell from an exact Euclidean
// distance transform. The robot is a disc, so a cell is traversable when the
// clearance there is larger than the robot radius plus a margin.
class CostMap {
 public:
  void build(const OccupancyMap& map, const CostMapConfig& config);
  void addObstacle(const Point2D& centre, double radius);
  void clearLearnedObstacles();

  bool isValid() const { return base_.isValid(); }
  const MapMeta& meta() const { return base_.meta(); }
  const CostMapConfig& config() const { return config_; }
  int width() const { return base_.width(); }
  int height() const { return base_.height(); }
  double hardClearance() const { return config_.robotRadius + config_.safetyMargin; }

  bool contains(int col, int row) const { return base_.contains(col, row); }
  bool isOccupied(int col, int row) const;     // map or learned obstacle
  double clearance(int col, int row) const;    // metres to the nearest obstacle, 0 outside
  double clearanceAt(const Point2D& p) const;  // the same at any point, interpolated between cell centres
  bool isTraversable(int col, int row) const;  // the robot centre may be here (hard clearance)
  bool isTraversable(int col, int row, double minClearance) const;
  double costFactor(int col, int row) const;   // >= 1, grows near obstacles

  bool worldToCell(const Point2D& p, int& col, int& row) const { return base_.worldToCell(p, col, row); }
  Point2D cellCenter(int col, int row) const { return base_.cellCenter(col, row); }

  const std::vector<LearnedObstacle>& learnedObstacles() const { return learned_; }

 private:
  std::size_t index(int col, int row) const {
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(width()) + static_cast<std::size_t>(col);
  }
  void resetOccupancy();
  void stamp(const LearnedObstacle& obstacle);
  void recomputeClearance();

  OccupancyMap base_;
  CostMapConfig config_;
  std::vector<std::uint8_t> occupied_;
  std::vector<float> clearance_;
  std::vector<LearnedObstacle> learned_;
};

}  // namespace vacuum
