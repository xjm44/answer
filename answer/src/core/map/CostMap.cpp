#include "core/map/CostMap.h"

#include <algorithm>
#include <cmath>

#include "core/common/Geometry.h"
#include "core/common/Log.h"

namespace vacuum {

namespace {

constexpr double kFar = 1e20;
// Clearances are stored as float: 0.19 m becomes 0.18999999. Without this tolerance, cells exactly
// at the limit (common: 10 cells x 2 cm - half a cell) would count as blocked and close narrow gaps.
constexpr double kClearanceEpsilon = 1e-4;

// 1D squared Euclidean distance transform (Felzenszwalb & Huttenlocher, 2012).
// f: input costs (0 at obstacles, kFar elsewhere); d: output; v, z: scratch (n and n+1).
void distanceTransform1d(const double* f, double* d, int n, int* v, double* z) {
  int k = 0;
  v[0] = 0;
  z[0] = -kFar;
  z[1] = kFar;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kFar;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    d[q] = double(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

}  // namespace

void CostMap::build(const OccupancyMap& map, const CostMapConfig& config) {
  base_ = map;
  config_ = config;
  learned_.clear();
  resetOccupancy();
  recomputeClearance();
  if (!isValid()) {
    VLOG_W(lcMap) << "built from an empty map";
    return;
  }
  std::size_t occupied = 0;
  std::size_t traversable = 0;
  for (int r = 0; r < height(); ++r) {
    for (int c = 0; c < width(); ++c) {
      if (occupied_[index(c, r)]) ++occupied;
      else if (isTraversable(c, r)) ++traversable;
    }
  }
  const std::size_t total = static_cast<std::size_t>(width()) * static_cast<std::size_t>(height());
  VLOG_I(lcMap) << "built " << width() << " x " << height() << " cells at " << meta().resolution * 100.0
                << " cm, origin " << meta().origin << ": " << occupied << " occupied, " << traversable
                << " traversable (" << QString::number(100.0 * traversable / total, 'f', 1)
                << " %); robot centre needs >= " << QString::number(hardClearance(), 'f', 3)
                << " m clearance, prefers " << config_.preferredClearance << " m";
}

void CostMap::addObstacle(const Point2D& centre, double radius) {
  if (!isValid()) return;
  learned_.push_back({centre, radius});
  stamp(learned_.back());
  recomputeClearance();
  VLOG_I(lcMap) << "learned obstacle #" << learned_.size() << " at " << centre << ", radius "
                << QString::number(radius, 'f', 3) << " m";
}

void CostMap::clearLearnedObstacles() {
  if (!isValid()) return;
  VLOG_I(lcMap) << "cleared " << learned_.size() << " learned obstacle(s)";
  learned_.clear();
  resetOccupancy();
  recomputeClearance();
}

void CostMap::resetOccupancy() {
  const auto& cells = base_.cells();
  occupied_.resize(cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i) occupied_[i] = cells[i] >= OccupancyMap::kOccupiedThreshold ? 1 : 0;
}

void CostMap::stamp(const LearnedObstacle& obstacle) {
  const double res = meta().resolution;
  int col = 0;
  int row = 0;
  base_.worldToCell(obstacle.centre, col, row);
  const int reach = static_cast<int>(std::ceil(obstacle.radius / res)) + 1;
  for (int r = row - reach; r <= row + reach; ++r) {
    for (int c = col - reach; c <= col + reach; ++c) {
      if (contains(c, r) && distance(cellCenter(c, r), obstacle.centre) <= obstacle.radius) occupied_[index(c, r)] = 1;
    }
  }
}

void CostMap::recomputeClearance() {
  const int w = width();
  const int h = height();
  const double res = meta().resolution;
  std::vector<double> grid(occupied_.size());
  for (std::size_t i = 0; i < grid.size(); ++i) grid[i] = occupied_[i] ? 0.0 : kFar;

  const int n = std::max(w, h);
  std::vector<double> f(n), d(n), z(n + 1);
  std::vector<int> v(n);
  for (int c = 0; c < w; ++c) {  // columns
    for (int r = 0; r < h; ++r) f[r] = grid[index(c, r)];
    distanceTransform1d(f.data(), d.data(), h, v.data(), z.data());
    for (int r = 0; r < h; ++r) grid[index(c, r)] = d[r];
  }
  for (int r = 0; r < h; ++r) {  // rows
    for (int c = 0; c < w; ++c) f[c] = grid[index(c, r)];
    distanceTransform1d(f.data(), d.data(), w, v.data(), z.data());
    for (int c = 0; c < w; ++c) grid[index(c, r)] = d[c];
  }

  // Centre-to-centre distance minus half a cell approximates the distance to the obstacle's edge.
  clearance_.resize(grid.size());
  for (std::size_t i = 0; i < grid.size(); ++i)
    clearance_[i] = occupied_[i] ? 0.0f : static_cast<float>(std::max(0.0, std::sqrt(grid[i]) * res - 0.5 * res));
}

bool CostMap::isOccupied(int col, int row) const { return !contains(col, row) || occupied_[index(col, row)] != 0; }

double CostMap::clearance(int col, int row) const { return contains(col, row) ? clearance_[index(col, row)] : 0.0; }

double CostMap::clearanceAt(const Point2D& p) const {
  if (!isValid()) return 0.0;
  // Continuous cell coordinates with cell centres at integers (row 0 is the top of the map).
  const double res = meta().resolution;
  const double fc = (p.x - meta().origin.x) / res - 0.5;
  const double fr = (height() - 1) - ((p.y - meta().origin.y) / res - 0.5);
  const int c0 = static_cast<int>(std::floor(fc));
  const int r0 = static_cast<int>(std::floor(fr));
  const double tx = fc - c0;
  const double ty = fr - r0;
  return (1.0 - tx) * (1.0 - ty) * clearance(c0, r0) + tx * (1.0 - ty) * clearance(c0 + 1, r0) +
         (1.0 - tx) * ty * clearance(c0, r0 + 1) + tx * ty * clearance(c0 + 1, r0 + 1);
}

bool CostMap::isTraversable(int col, int row) const { return isTraversable(col, row, hardClearance()); }

bool CostMap::isTraversable(int col, int row, double minClearance) const {
  return contains(col, row) && !occupied_[index(col, row)] &&
         clearance_[index(col, row)] >= minClearance - kClearanceEpsilon;
}

double CostMap::costFactor(int col, int row) const {
  const double c = clearance(col, row);
  const double preferred = config_.preferredClearance;
  if (c >= preferred) return 1.0;
  const double t = std::clamp((preferred - c) / (preferred - hardClearance()), 0.0, 1.0);
  return 1.0 + config_.clearanceWeight * t * t;
}

}  // namespace vacuum
