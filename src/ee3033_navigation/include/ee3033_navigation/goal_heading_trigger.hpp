// SPDX-License-Identifier: Apache-2.0
#ifndef EE3033_NAVIGATION__GOAL_HEADING_TRIGGER_HPP_
#define EE3033_NAVIGATION__GOAL_HEADING_TRIGGER_HPP_
#include <cmath>
#include <stdexcept>

namespace ee3033_navigation
{
inline bool actual_goal_within_xy_tolerance(double distance, double tolerance)
{
  if (!std::isfinite(distance) || distance < 0. ||
    !std::isfinite(tolerance) || tolerance <= 0.)
  {
    throw std::invalid_argument("Fresh actual goal distance and XY tolerance are required");
  }
  return distance <= tolerance;
}
}  // namespace ee3033_navigation
#endif  // EE3033_NAVIGATION__GOAL_HEADING_TRIGGER_HPP_
