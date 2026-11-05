// SPDX-License-Identifier: Apache-2.0
#ifndef EE3033_NAVIGATION__RAW_COST_CLEARANCE_HPP_
#define EE3033_NAVIGATION__RAW_COST_CLEARANCE_HPP_
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace ee3033_navigation
{
// Caller holds the costmap mutex. Distances are to closed raw cell squares,
// NOT to inflated253 boundaries, and do not subtract any robot footprint.
inline bool raw_obstacle_near(
  const unsigned char * cells, std::size_t width, std::size_t height,
  double resolution, double origin_x, double origin_y,
  double x, double y, double radius = 0.22)
{
  if (!cells || !width || !height || !std::isfinite(resolution) || resolution <= 0. ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) || !std::isfinite(x) ||
    !std::isfinite(y) || !std::isfinite(radius) || radius <= 0.)
  {
    throw std::invalid_argument("Valid raw costmap and finite query geometry are required");
  }
  // Outside the local map is unknown: select conservative tracking rather
  // than infer wide clearance from absent cells. Motion checks remain native.
  if (x - radius <= origin_x || y - radius <= origin_y ||
    x + radius >= origin_x + width * resolution ||
    y + radius >= origin_y + height * resolution)
  {
    return true;
  }
  const auto lower_col = static_cast<std::size_t>(std::floor((x - radius - origin_x) / resolution));
  const auto lower_row = static_cast<std::size_t>(std::floor((y - radius - origin_y) / resolution));
  // Closed squares on the lower boundary can end exactly at x/y-radius.
  // Include the preceding row/column, then let the exact AABB test decide.
  const auto c0 = lower_col > 0 ? lower_col - 1 : 0;
  const auto r0 = lower_row > 0 ? lower_row - 1 : 0;
  // Decimal division can round an upper grid-boundary index just below an
  // integer. Include one following row/column; the exact test filters extras.
  const auto c1 = std::min(width - 1,
    static_cast<std::size_t>(std::floor((x + radius - origin_x) / resolution)) + 1);
  const auto r1 = std::min(height - 1,
    static_cast<std::size_t>(std::floor((y + radius - origin_y) / resolution)) + 1);
  for (std::size_t r = r0; r <= r1; ++r) {
    for (std::size_t c = c0; c <= c1; ++c) {
      if (cells[r * width + c] < 254) {
        continue;
      }
      const double left = origin_x + c * resolution;
      const double bottom = origin_y + r * resolution;
      const double dx = std::max({left - x, x - left - resolution, 0.});
      const double dy = std::max({bottom - y, y - bottom - resolution, 0.});
      if (dx * dx + dy * dy <= radius * radius + 1e-12) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace ee3033_navigation
#endif  // EE3033_NAVIGATION__RAW_COST_CLEARANCE_HPP_
