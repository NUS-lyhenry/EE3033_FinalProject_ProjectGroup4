// SPDX-License-Identifier: Apache-2.0
#ifndef EE3033_NAVIGATION__REGULATED_PATH_FOLLOWER_HPP_
#define EE3033_NAVIGATION__REGULATED_PATH_FOLLOWER_HPP_

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ee3033_navigation
{

// Keep all path transformation, regulation and collision checks in native RPP.
// Its angular ramp uses command history rather than potentially zero reported
// twist; strict final rotations brake from real pose changes before success.
class RegulatedPathFollower final
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  using Clock = std::chrono::steady_clock;
  using Base = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
  void reset_reference();
  void observe_yaw(const geometry_msgs::msg::PoseStamped & pose);
  void brake_final_rotation(
    geometry_msgs::msg::TwistStamped & command,
    const geometry_msgs::msg::PoseStamped & pose, nav2_core::GoalChecker * goal_checker,
    const Clock::time_point & now, double step);
  rcl_interfaces::msg::SetParametersResult validate_caps(
    const std::vector<rclcpp::Parameter> & parameters) const;

  std::mutex reference_mutex_;
  std::atomic<bool> reset_requested_{true};
  bool have_reference_{false};
  double last_angular_command_{0.};
  Clock::time_point last_command_time_{};
  nav_msgs::msg::Path last_unpruned_plan_;
  bool have_yaw_sample_{false};
  bool have_yaw_rate_{false};
  double last_yaw_stamp_{0.};
  double last_pose_yaw_{0.};
  double measured_yaw_rate_{0.};
  double yaw_sample_interval_{0.};
  std::string yaw_frame_;
  std::deque<std::pair<Clock::time_point, double>> angular_history_;
  bool final_braking_{false};
  bool final_braking_failed_{false};
  unsigned int final_corrections_{0};
  Clock::time_point final_brake_started_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr caps_callback_;
};

}  // namespace ee3033_navigation
#endif  // EE3033_NAVIGATION__REGULATED_PATH_FOLLOWER_HPP_
