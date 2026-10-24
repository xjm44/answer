#pragma once

#include <string>
#include <vector>

#include "core/common/Types.h"
#include "core/map/CostMap.h"

namespace vacuum {

struct PlanResult {
  bool ok = false;
  std::string error;
  std::vector<Point2D> waypoints;  // world coordinates; first = start, last = goal
  double length = 0.0;             // metres
  bool tight = false;              // only found with the reduced safety margin
};

// Grid path planner: A* over the traversable cells of a CostMap (8-connected, no
// corner cutting), with step costs that grow near obstacles so paths keep to the
// middle of free space. The cell path is then shortened to a few waypoints with
// straight segments that are never closer to obstacles than the original path.
//
// If there is no path with the cost map's safety margin (robot radius + 2 cm), it plans again
// with a small margin: the robot fits through any gap wider than itself, and it may have got
// into a tight spot (e.g. between chair legs) that the normal margin would never let it leave.
class PathPlanner {
 public:
  struct Options {
    double startSearchRadius = 0.6;  // start in a no-go cell (e.g. after a bump): leave via nearest free cell
    double goalSearchRadius = 0.3;   // goal close to furniture: plan to nearest free cell, then go straight
    bool allowTight = true;          // retry with `tightMargin` when the normal margin finds no path
    double tightMargin = 0.005;      // m beyond the robot radius
  };

  static PlanResult plan(const CostMap& map, const Point2D& start, const Point2D& goal);
  static PlanResult plan(const CostMap& map, const Point2D& start, const Point2D& goal, const Options& options);

 private:
  static PlanResult planImpl(const CostMap& map, const Point2D& start, const Point2D& goal, const Options& options,
                             double minClearance);
};

}  // namespace vacuum
