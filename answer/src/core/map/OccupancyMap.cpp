#include "core/map/OccupancyMap.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

#include <QImage>

namespace vacuum {

OccupancyMap::OccupancyMap(const MapMeta& meta, std::vector<std::uint8_t> cells)
    : meta_(meta), cells_(std::move(cells)) {
  if (meta_.width <= 0 || meta_.height <= 0 || meta_.resolution <= 0.0)
    throw std::invalid_argument("OccupancyMap: invalid map header");
  if (cells_.size() != static_cast<std::size_t>(meta_.width) * static_cast<std::size_t>(meta_.height))
    throw std::invalid_argument("OccupancyMap: cell count does not match width x height");
}

OccupancyMap OccupancyMap::fromPng(const MapMeta& meta, const std::string& png) {
  QImage image;
  if (!image.loadFromData(reinterpret_cast<const uchar*>(png.data()), static_cast<int>(png.size()), "PNG"))
    throw std::runtime_error("map PNG could not be decoded");
  image = image.convertToFormat(QImage::Format_Grayscale8);
  if (image.width() != meta.width || image.height() != meta.height)
    throw std::runtime_error("map PNG size does not match the map header");

  const auto w = static_cast<std::size_t>(meta.width);
  std::vector<std::uint8_t> cells(w * static_cast<std::size_t>(meta.height));
  for (int row = 0; row < meta.height; ++row)  // scan lines are 4-byte aligned, copy row by row
    std::memcpy(&cells[static_cast<std::size_t>(row) * w], image.constScanLine(row), w);
  return OccupancyMap(meta, std::move(cells));
}

bool OccupancyMap::contains(int col, int row) const {
  return col >= 0 && row >= 0 && col < meta_.width && row < meta_.height;
}

std::uint8_t OccupancyMap::value(int col, int row) const {
  return cells_[static_cast<std::size_t>(row) * static_cast<std::size_t>(meta_.width) +
                static_cast<std::size_t>(col)];
}

bool OccupancyMap::isOccupied(int col, int row) const {
  return !contains(col, row) || value(col, row) >= kOccupiedThreshold;
}

Point2D OccupancyMap::cellCenter(int col, int row) const {
  // Spec section 7: x = ox + (c + 0.5) res, y = oy + (H - 1 - r + 0.5) res
  return {meta_.origin.x + (col + 0.5) * meta_.resolution,
          meta_.origin.y + (meta_.height - 1 - row + 0.5) * meta_.resolution};
}

bool OccupancyMap::worldToCell(const Point2D& p, int& col, int& row) const {
  col = static_cast<int>(std::floor((p.x - meta_.origin.x) / meta_.resolution));
  row = meta_.height - 1 - static_cast<int>(std::floor((p.y - meta_.origin.y) / meta_.resolution));
  return contains(col, row);
}

}  // namespace vacuum
