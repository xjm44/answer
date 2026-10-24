#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/common/Types.h"

namespace vacuum {

// 2D occupancy grid as delivered by get_map.
//
// Cells are stored row-major exactly like the PNG: row 0 is the top of the image
// (largest y), columns grow towards +x. Values: 0 = free, 255 = occupied.
class OccupancyMap {
 public:
  static constexpr std::uint8_t kOccupiedThreshold = 128;

  OccupancyMap() = default;
  OccupancyMap(const MapMeta& meta, std::vector<std::uint8_t> cells);

  // Decodes the 8-bit grey PNG frame of get_map. Throws std::runtime_error on failure.
  static OccupancyMap fromPng(const MapMeta& meta, const std::string& png);

  bool isValid() const { return meta_.width > 0 && meta_.height > 0; }
  const MapMeta& meta() const { return meta_; }
  int width() const { return meta_.width; }
  int height() const { return meta_.height; }
  double resolution() const { return meta_.resolution; }
  const std::vector<std::uint8_t>& cells() const { return cells_; }

  bool contains(int col, int row) const;
  std::uint8_t value(int col, int row) const;  // requires contains(col, row)
  bool isOccupied(int col, int row) const;     // cells outside the map count as occupied

  // Centre of a cell in world coordinates.
  Point2D cellCenter(int col, int row) const;
  // Cell containing a world point; returns false if the point is outside the map.
  bool worldToCell(const Point2D& p, int& col, int& row) const;

 private:
  MapMeta meta_;
  std::vector<std::uint8_t> cells_;
};

}  // namespace vacuum
