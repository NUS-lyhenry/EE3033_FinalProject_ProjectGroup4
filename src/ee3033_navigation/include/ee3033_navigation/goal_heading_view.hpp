// SPDX-License-Identifier: Apache-2.0
#ifndef EE3033_NAVIGATION__GOAL_HEADING_VIEW_HPP_
#define EE3033_NAVIGATION__GOAL_HEADING_VIEW_HPP_
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "ee3033_navigation/goal_heading_trigger.hpp"

namespace ee3033_navigation
{
// A short-lived view passed only to native RPP. ControllerServer retains its
// original checker and every real acceptance tolerance/state. Humble RPP uses
// getTolerances().position.x as a CARROT-distance terminal-heading trigger.
class GoalHeadingView final : public nav2_core::GoalChecker
{
public:
  GoalHeadingView(nav2_core::GoalChecker & original, double actual_goal_distance)
  : original_(original), actual_goal_distance_(actual_goal_distance) {}

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    original_.initialize(parent, name, costmap);
  }
  void reset() override {original_.reset();}
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override
  {
    return original_.isGoalReached(query, goal, velocity);
  }
  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    if (!original_.getTolerances(pose, velocity)) {
      throw nav2_core::PlannerException("Goal-checker tolerance is unavailable");
    }
    try {
      if (!actual_goal_within_xy_tolerance(actual_goal_distance_, pose.position.x)) {
        pose.position.x = 0.;
        pose.position.y = 0.;
      }
    } catch (const std::invalid_argument & error) {
      throw nav2_core::PlannerException(error.what());
    }
    return true;
  }

private:
  nav2_core::GoalChecker & original_;
  double actual_goal_distance_;
};
}  // namespace ee3033_navigation
#endif  // EE3033_NAVIGATION__GOAL_HEADING_VIEW_HPP_
