// SPDX-License-Identifier: Apache-2.0
#ifndef EE3033_NAVIGATION__CORNER_TRACKING_HPP_
#define EE3033_NAVIGATION__CORNER_TRACKING_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ee3033_navigation
{

struct PathPoint
{
  double x;
  double y;
};

// Cache features from the COMPLETE plan, so a later corner is not missed when
// replanning is delayed. Direction windows suppress raster stair-step noise.
// Neither feature extraction nor proximity selection edits the path geometry.
inline std::vector<PathPoint> sharp_bend_features(const std::vector<PathPoint> & points)
{
  std::vector<PathPoint> features;
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("Path coordinates must be finite");
    }
  }
  if (points.size() < 3) {
    return features;
  }
  std::vector<double> distance(points.size(), 0.);
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      distance[i] = distance[i - 1] + std::hypot(
        points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
  }
  constexpr double direction_window = 0.08;
  constexpr double minimum_span = 0.04;
  constexpr double bend_threshold = 0.55;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    auto before = std::upper_bound(
      distance.begin(), distance.begin() + i, distance[i] - direction_window);
    const auto j = before == distance.begin() ? 0U :
      static_cast<std::size_t>(before - distance.begin() - 1);
    const auto after = std::lower_bound(
      distance.begin() + i + 1, distance.end(), distance[i] + direction_window);
    const auto k = after == distance.end() ? points.size() - 1 :
      static_cast<std::size_t>(after - distance.begin());
    if (distance[i] - distance[j] < minimum_span || distance[k] - distance[i] < minimum_span) {
      continue;
    }
    const double ax = points[i].x - points[j].x;
    const double ay = points[i].y - points[j].y;
    const double bx = points[k].x - points[i].x;
    const double by = points[k].y - points[i].y;
    if (std::hypot(ax, ay) < 1e-9 || std::hypot(bx, by) < 1e-9) {
      continue;
    }
    if (std::abs(std::atan2(ax * by - ay * bx, ax * bx + ay * by)) > bend_threshold) {
      features.push_back(points[i]);
    }
  }
  return features;
}

inline bool has_sharp_bend(const std::vector<PathPoint> & points)
{
  try {
    return !sharp_bend_features(points).empty();
  } catch (const std::invalid_argument &) {
    return true;
  }
}

inline bool near_sharp_bend(
  const std::vector<PathPoint> & features, const PathPoint & position, double radius = 0.30)
{
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(radius) || radius <= 0.)
  {
    throw std::invalid_argument("Finite physical position and corner proximity are required");
  }
  for (const auto & feature : features) {
    if (!std::isfinite(feature.x) || !std::isfinite(feature.y)) {
      throw std::invalid_argument("Cached corner coordinates must be finite");
    }
    if (std::hypot(feature.x - position.x, feature.y - position.y) <= radius) {
      return true;
    }
  }
  return false;
}

}  // namespace ee3033_navigation
#endif  // EE3033_NAVIGATION__CORNER_TRACKING_HPP_
