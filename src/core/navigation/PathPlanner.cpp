#include "core/navigation/PathPlanner.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include <QElapsedTimer>

#include "core/common/Geometry.h"
#include "core/common/Log.h"

namespace vacuum {

namespace {

constexpr double kSqrt2 = 1.41421356237309504880;

struct Cell {
  int col = 0;
  int row = 0;
  bool operator==(const Cell& o) const { return col == o.col && row == o.row; }
};

// Breadth-first search through non-occupied cells for the nearest traversable cell.
// Searching through free space (rather than any cell in a radius) never "escapes" through a wall.
bool nearestTraversable(const CostMap& map, const Cell& from, double radius, double minClearance, Cell& out) {
  if (map.isTraversable(from.col, from.row, minClearance)) {
    out = from;
    return true;
  }
  if (!map.contains(from.col, from.row)) return false;
  const int w = map.width();
  const int maxCells = static_cast<int>(std::ceil(radius / map.meta().resolution));
  std::vector<std::uint8_t> seen(static_cast<std::size_t>(w) * map.height(), 0);
  std::deque<Cell> queue{from};
  seen[static_cast<std::size_t>(from.row) * w + from.col] = 1;
  static const int kDc[4] = {1, -1, 0, 0};
  static const int kDr[4] = {0, 0, 1, -1};
  while (!queue.empty()) {
    const Cell cell = queue.front();
    queue.pop_front();
    for (int k = 0; k < 4; ++k) {
      const Cell next{cell.col + kDc[k], cell.row + kDr[k]};
      if (!map.contains(next.col, next.row)) continue;
      if (std::abs(next.col - from.col) > maxCells || std::abs(next.row - from.row) > maxCells) continue;
      auto& flag = seen[static_cast<std::size_t>(next.row) * w + next.col];
      if (flag) continue;
      flag = 1;
      if (map.isTraversable(next.col, next.row, minClearance)) {
        out = next;
        return true;
      }
      // Allow passing through occupied cells only at the very start (robot overlapping a learned obstacle).
      if (map.isOccupied(next.col, next.row) && !map.isOccupied(from.col, from.row)) continue;
      queue.push_back(next);
    }
  }
  return false;
}

// True if the straight line a->b only crosses traversable cells and keeps at least `minClearance`
// everywhere. Clearance is checked at the points of the line itself (interpolated), not at the
// cells it touches: a line can pass up to half a cell off a cell centre, which matters in gaps
// only a cell or two wide.
bool segmentIsClear(const CostMap& map, const Cell& a, const Cell& b, double minClearance, double hardLimit) {
  const double dc = b.col - a.col;
  const double dr = b.row - a.row;
  const Point2D pa = map.cellCenter(a.col, a.row);
  const Point2D pb = map.cellCenter(b.col, b.row);
  const int steps = std::max(1, static_cast<int>(std::ceil(2.0 * std::max(std::abs(dc), std::abs(dr)))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const int c = static_cast<int>(std::lround(a.col + t * dc));
    const int r = static_cast<int>(std::lround(a.row + t * dr));
    if (!map.isTraversable(c, r, hardLimit)) return false;
    if (map.clearanceAt({pa.x + t * (pb.x - pa.x), pa.y + t * (pb.y - pa.y)}) < minClearance) return false;
  }
  return true;
}

// Greedy shortcutting: from each anchor, jump to the furthest later cell that can be reached
// in a straight line without getting closer to obstacles than the cell path did.
std::vector<Cell> shortcut(const CostMap& map, const std::vector<Cell>& cells, double hardLimit) {
  if (cells.size() <= 2) return cells;
  constexpr double tolerance = 0.002;  // m a shortcut may come closer to obstacles than the cell path
  const double preferred = map.config().preferredClearance;
  std::vector<Cell> out{cells.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    std::size_t best = anchor + 1;
    double pathClearance = std::min(map.clearance(cells[anchor].col, cells[anchor].row), preferred);
    for (std::size_t j = anchor + 1; j < cells.size(); ++j) {
      pathClearance = std::min(pathClearance, map.clearance(cells[j].col, cells[j].row));
      if (segmentIsClear(map, cells[anchor], cells[j], pathClearance - tolerance, hardLimit)) best = j;
      else if (j - best > 50) break;  // far past the last visible cell; stop looking
    }
    out.push_back(cells[best]);
    anchor = best;
  }
  return out;
}

}  // namespace

PlanResult PathPlanner::plan(const CostMap& map, const Point2D& start, const Point2D& goal) {
  return plan(map, start, goal, Options{});
}

PlanResult PathPlanner::plan(const CostMap& map, const Point2D& start, const Point2D& goal, const Options& options) {
  PlanResult result = planImpl(map, start, goal, options, map.hardClearance());
  if (!result.ok && options.allowTight && map.isValid()) {
    const double tight = map.config().robotRadius + options.tightMargin;
    VLOG_W(lcPlan) << "no path with " << QString::number(map.hardClearance(), 'f', 3) << " m clearance ("
                   << result.error.c_str() << "); trying the tight margin, " << QString::number(tight, 'f', 3) << " m";
    PlanResult tightResult = planImpl(map, start, goal, options, tight);
    if (tightResult.ok) {
      tightResult.tight = true;
      result = std::move(tightResult);
      VLOG_W(lcPlan) << "TIGHT path found: parts of it pass obstacles with only "
                     << QString::number(options.tightMargin * 100.0, 'f', 1) << " cm to spare";
    }
  }
  if (result.ok) {
    QString points;
    for (const Point2D& p : result.waypoints) points += str(p) + QLatin1Char(' ');
    VLOG_D(lcPlan) << "waypoints: " << points;
  } else {
    VLOG_W(lcPlan) << "FAILED from " << start << " to " << goal << ": " << result.error.c_str() << " ("
                   << map.learnedObstacles().size() << " learned obstacle(s) on the cost map)";
  }
  return result;
}

PlanResult PathPlanner::planImpl(const CostMap& map, const Point2D& start, const Point2D& goal,
                                 const Options& options, double minClearance) {
  QElapsedTimer clock;
  clock.start();
  PlanResult result;
  if (!map.isValid()) {
    result.error = "no map";
    return result;
  }

  Cell startPos;
  Cell goalPos;
  if (!map.worldToCell(start, startPos.col, startPos.row)) {
    result.error = "robot is outside the map";
    return result;
  }
  if (!map.worldToCell(goal, goalPos.col, goalPos.row)) {
    result.error = "goal is outside the map";
    return result;
  }
  VLOG_I(lcPlan) << "plan from " << start << " to " << goal << ", robot centre clearance >= "
                 << QString::number(minClearance, 'f', 3) << " m";
  auto describe = [&](const char* what, const Cell& cell) {
    VLOG_D(lcPlan) << what << " cell (" << cell.col << ", " << cell.row << "): clearance "
                   << QString::number(map.clearance(cell.col, cell.row), 'f', 3) << " m, "
                   << (map.isOccupied(cell.col, cell.row)
                           ? "OCCUPIED"
                           : (map.isTraversable(cell.col, cell.row, minClearance) ? "traversable" : "in the no-go zone"));
  };
  describe("start", startPos);
  describe("goal", goalPos);

  Cell startCell;
  Cell goalCell;
  if (!nearestTraversable(map, startPos, options.startSearchRadius, minClearance, startCell)) {
    result.error = "robot is boxed in: no free space within reach";
    return result;
  }
  if (!nearestTraversable(map, goalPos, options.goalSearchRadius, minClearance, goalCell)) {
    result.error = "goal is blocked by obstacles";
    return result;
  }
  if (!(startCell == startPos))
    VLOG_I(lcPlan) << "start is in the no-go zone: leaving via " << map.cellCenter(startCell.col, startCell.row)
                   << ", " << QString::number(distance(start, map.cellCenter(startCell.col, startCell.row)), 'f', 3)
                   << " m away";
  if (!(goalCell == goalPos))
    VLOG_I(lcPlan) << "goal is in the no-go zone: planning to " << map.cellCenter(goalCell.col, goalCell.row)
                   << ", then straight to the goal";

  // --- A* -------------------------------------------------------------------
  const int w = map.width();
  const std::size_t n = static_cast<std::size_t>(w) * map.height();
  const double res = map.meta().resolution;
  auto indexOf = [w](int c, int r) { return static_cast<std::size_t>(r) * w + c; };
  auto heuristic = [&](int c, int r) {  // octile distance, admissible because cost factors are >= 1
    const int dx = std::abs(c - goalCell.col);
    const int dy = std::abs(r - goalCell.row);
    return res * ((dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy));
  };

  constexpr double kUnreached = std::numeric_limits<double>::infinity();
  std::vector<double> cost(n, kUnreached);
  std::vector<std::int64_t> parent(n, -1);
  std::vector<std::uint8_t> closed(n, 0);
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  const std::size_t startIndex = indexOf(startCell.col, startCell.row);
  const std::size_t goalIndex = indexOf(goalCell.col, goalCell.row);
  cost[startIndex] = 0.0;
  open.push({heuristic(startCell.col, startCell.row), startIndex});

  static const int kDc[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDr[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!open.empty()) {
    const std::size_t i = open.top().second;
    open.pop();
    if (closed[i]) continue;
    closed[i] = 1;
    if (i == goalIndex) break;
    const int c = static_cast<int>(i % w);
    const int r = static_cast<int>(i / w);
    const double here = map.costFactor(c, r);
    for (int k = 0; k < 8; ++k) {
      const int nc = c + kDc[k];
      const int nr = r + kDr[k];
      if (!map.isTraversable(nc, nr, minClearance)) continue;
      const bool diagonal = kDc[k] != 0 && kDr[k] != 0;
      if (diagonal && (!map.isTraversable(nc, r, minClearance) || !map.isTraversable(c, nr, minClearance)))
        continue;  // no corner cutting
      const std::size_t j = indexOf(nc, nr);
      if (closed[j]) continue;
      const double step = (diagonal ? kSqrt2 : 1.0) * res * 0.5 * (here + map.costFactor(nc, nr));
      const double candidate = cost[i] + step;
      if (candidate < cost[j]) {
        cost[j] = candidate;
        parent[j] = static_cast<std::int64_t>(i);
        open.push({candidate + heuristic(nc, nr), j});
      }
    }
  }
  const auto expanded = std::count(closed.begin(), closed.end(), std::uint8_t{1});
  if (!closed[goalIndex]) {
    result.error = "no collision-free path to the goal (searched " + std::to_string(expanded) +
                   " cells: start and goal are not connected through traversable space)";
    return result;
  }

  std::vector<Cell> cells;
  for (std::int64_t i = static_cast<std::int64_t>(goalIndex); i >= 0; i = parent[static_cast<std::size_t>(i)])
    cells.push_back({static_cast<int>(i % w), static_cast<int>(i / w)});
  std::reverse(cells.begin(), cells.end());
  cells = shortcut(map, cells, minClearance);

  // --- Waypoints: exact start, smoothed cells, exact goal --------------------
  result.waypoints.push_back(start);
  for (std::size_t k = 0; k < cells.size(); ++k) {
    if (k == 0 && startCell == startPos) continue;  // the start itself is in this cell
    if (k + 1 == cells.size() && goalCell == goalPos) continue;
    result.waypoints.push_back(map.cellCenter(cells[k].col, cells[k].row));
  }
  result.waypoints.push_back(goal);

  // Drop near-duplicates and measure.
  std::vector<Point2D> unique{result.waypoints.front()};
  for (std::size_t k = 1; k < result.waypoints.size(); ++k) {
    if (distance(unique.back(), result.waypoints[k]) > 1e-3) unique.push_back(result.waypoints[k]);
  }
  if (unique.size() == 1) unique.push_back(goal);
  result.waypoints = std::move(unique);
  for (std::size_t k = 1; k < result.waypoints.size(); ++k)
    result.length += distance(result.waypoints[k - 1], result.waypoints[k]);
  result.ok = true;
  VLOG_I(lcPlan) << "path found: " << QString::number(result.length, 'f', 2) << " m, " << result.waypoints.size()
                 << " waypoints (A* " << cells.size() << " cells after shortcutting, " << expanded
                 << " cells expanded) in " << clock.elapsed() << " ms";
  return result;
}

}  // namespace vacuum
