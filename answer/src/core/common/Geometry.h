#pragma once

#include <cmath>

#include "core/common/Types.h"

namespace vacuum {

inline constexpr double kPi = 3.14159265358979323846;

inline constexpr double degToRad(double deg) { return deg * kPi / 180.0; }
inline constexpr double radToDeg(double rad) { return rad * 180.0 / kPi; }

// Wraps an angle to [-pi, pi).
inline double wrapAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}

inline double distance(const Point2D& a, const Point2D& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// Closest point to p on segment ab.
inline Point2D projectOntoSegment(const Point2D& p, const Point2D& a, const Point2D& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 <= 1e-12) return a;
  double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
  t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
  return {a.x + t * dx, a.y + t * dy};
}

}  // namespace vacuum
