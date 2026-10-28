// SPDX-License-Identifier: Apache-2.0
#include "ee3033_navigation/regulated_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ee3033_navigation
{

void RegulatedPathFollower::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  Base::configure(parent, name, std::move(tf), std::move(costmap_ros));
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Navigation controller node disappeared");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_linear_vel", rclcpp::ParameterValue(0.12));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_angular_vel", rclcpp::ParameterValue(0.40));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotation_command_delay", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotation_braking_accel", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotation_stop_margin", rclcpp::ParameterValue(0.005));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rotation_motor_deadband", rclcpp::ParameterValue(0.08));
  const auto valid = validate_caps({
    node->get_parameter(name + ".max_linear_vel"),
    node->get_parameter(name + ".max_angular_vel"),
    node->get_parameter(name + ".rotation_command_delay"),
    node->get_parameter(name + ".rotation_braking_accel"),
    node->get_parameter(name + ".rotation_stop_margin"),
    node->get_parameter(name + ".rotation_motor_deadband")});
  if (!valid.successful) {
    throw std::invalid_argument(valid.reason);
  }
  // Validate only. Read committed values in computeVelocityCommands so a
  // rejected atomic ROS update cannot leave hidden, partially updated caps.
  caps_callback_ = node->add_on_set_parameters_callback(
    std::bind(&RegulatedPathFollower::validate_caps, this, std::placeholders::_1));
  drive_subscription_ = node->create_subscription<std_msgs::msg::Bool>(
    "/project/drive_enabled", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      if (!message->data) {
        reset_requested_.store(true);
      }
    });
  reset_requested_.store(true);
}

rcl_interfaces::msg::SetParametersResult RegulatedPathFollower::validate_caps(
  const std::vector<rclcpp::Parameter> & parameters) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    const bool cap = name == plugin_name_ + ".max_linear_vel" ||
      name == plugin_name_ + ".max_angular_vel";
    const bool delay = name == plugin_name_ + ".rotation_command_delay";
    const bool brake = name == plugin_name_ + ".rotation_braking_accel";
    const bool margin = name == plugin_name_ + ".rotation_stop_margin";
    const bool deadband = name == plugin_name_ + ".rotation_motor_deadband";
    if (!(cap || delay || brake || margin || deadband)) {
      continue;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
    } else {
      const double value = parameter.as_double();
      result.successful = std::isfinite(value) &&
        (cap ? value > 0. : delay ? value >= 0. && value <= 1. :
        brake ? value > 0. && value <= 0.8 : margin ? value > 0. && value <= 0.02 :
        value >= 0. && value < 0.4);
    }
    if (!result.successful) {
      result.reason = "Controller caps and rotation braking parameters are outside physical bounds";
      break;
    }
  }
  return result;
}

void RegulatedPathFollower::reset_reference()
{
  have_reference_ = false;
  last_angular_command_ = 0.;
  have_yaw_sample_ = have_yaw_rate_ = false;
  measured_yaw_rate_ = 0.;
  yaw_sample_interval_ = 0.;
  angular_history_.clear();
  final_braking_ = false;
  final_braking_failed_ = false;
  final_corrections_ = 0;
}

void RegulatedPathFollower::activate()
{
  std::lock_guard<std::mutex> lock(reference_mutex_);
  reset_reference();
  reset_requested_.store(true);
  Base::activate();
  // RPP installs its dynamic callback during activate. Register our validator
  // after it so a bad cap rejects an atomic update before RPP mutates any of
  // its own cached values. ROS invokes the newest callback first.
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Navigation controller node disappeared");
  }
  caps_callback_.reset();
  caps_callback_ = node->add_on_set_parameters_callback(
    std::bind(&RegulatedPathFollower::validate_caps, this, std::placeholders::_1));
}

void RegulatedPathFollower::deactivate()
{
  std::lock_guard<std::mutex> lock(reference_mutex_);
  reset_reference();
  reset_requested_.store(true);
  Base::deactivate();
}

void RegulatedPathFollower::cleanup()
{
  std::lock_guard<std::mutex> lock(reference_mutex_);
  reset_reference();
  last_unpruned_plan_ = nav_msgs::msg::Path();
  caps_callback_.reset();
  drive_subscription_.reset();
  Base::cleanup();
}

void RegulatedPathFollower::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(reference_mutex_);
  bool changed_goal = path.header.frame_id != last_unpruned_plan_.header.frame_id ||
    path.poses.empty() || last_unpruned_plan_.poses.empty();
  if (!changed_goal) {
    const auto & a = last_unpruned_plan_.poses.back().pose;
    const auto & b = path.poses.back().pose;
    const double delta = tf2::getYaw(a.orientation) - tf2::getYaw(b.orientation);
    changed_goal = std::hypot(a.position.x - b.position.x, a.position.y - b.position.y) > 0.10 ||
      std::abs(std::atan2(std::sin(delta), std::cos(delta))) > 0.10;
  }
  if (changed_goal) {
    reset_reference();
  }
  // A normal one-Hz replan to the same goal must not restart the ramp. Stop,
  // lifecycle transitions and command gaps independently invalidate references.
  last_unpruned_plan_ = path;
  Base::setPlan(path);
}

void RegulatedPathFollower::observe_yaw(const geometry_msgs::msg::PoseStamped & pose)
{
  const double stamp = rclcpp::Time(pose.header.stamp).seconds();
  const double yaw = tf2::getYaw(pose.pose.orientation);
  if (have_yaw_sample_ && pose.header.frame_id == yaw_frame_) {
    const double dt = stamp - last_yaw_stamp_;
    if (dt > 1e-4 && dt <= 0.5) {
      const double delta = yaw - last_pose_yaw_;
      measured_yaw_rate_ = std::atan2(std::sin(delta), std::cos(delta)) / dt;
      yaw_sample_interval_ = dt;
      have_yaw_rate_ = std::isfinite(measured_yaw_rate_);
    } else if (dt < 0. || dt > 0.5) {
      have_yaw_rate_ = false;
    }
  }
  if (!have_yaw_sample_ || stamp != last_yaw_stamp_ || pose.header.frame_id != yaw_frame_) {
    last_yaw_stamp_ = stamp;
    last_pose_yaw_ = yaw;
    yaw_frame_ = pose.header.frame_id;
    have_yaw_sample_ = true;
  }
}

void RegulatedPathFollower::brake_final_rotation(
  geometry_msgs::msg::TwistStamped & command,
  const geometry_msgs::msg::PoseStamped & pose, nav2_core::GoalChecker * goal_checker,
  const Clock::time_point & now, double step)
{
  // Only slow an already collision-checked PURE rotation. Translating commands
  // retain their native curvature; no altered pose or synthetic odometry is
  // supplied to RPP. A circular body's swept footprint is unchanged by a slower
  // pure rotation, including the braking hold at zero.
  if (std::abs(command.twist.linear.x) > 1e-6 || !goal_checker ||
    last_unpruned_plan_.poses.empty())
  {
    final_braking_ = false;
    return;
  }
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    return;
  }
  const double xy_tolerance = pose_tolerance.position.x;
  const double yaw_tolerance = std::abs(tf2::getYaw(pose_tolerance.orientation));
  if (!std::isfinite(xy_tolerance) || xy_tolerance <= 0. ||
    !std::isfinite(yaw_tolerance) || yaw_tolerance >= 1.5)
  {
    final_braking_ = false;
    return;
  }
  auto goal = last_unpruned_plan_.poses.back();
  goal.header.frame_id = last_unpruned_plan_.header.frame_id;
  goal.header.stamp = pose.header.stamp;
  geometry_msgs::msg::PoseStamped transformed;
  if (!transformPose(pose.header.frame_id, goal, transformed)) {
    throw nav2_core::PlannerException("Cannot transform final heading for braking");
  }
  if (std::hypot(transformed.pose.position.x - pose.pose.position.x,
    transformed.pose.position.y - pose.pose.position.y) > xy_tolerance + 1e-4)
  {
    final_braking_ = false;
    return;
  }
  const double delta = tf2::getYaw(transformed.pose.orientation) - tf2::getYaw(pose.pose.orientation);
  const double error = std::atan2(std::sin(delta), std::cos(delta));
  const double remaining = std::abs(error);
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Controller node disappeared during braking");
  }
  const double wmax = node->get_parameter(plugin_name_ + ".max_angular_vel").as_double();
  const double acceleration = std::min(
    node->get_parameter(plugin_name_ + ".max_angular_accel").as_double(),
    node->get_parameter(plugin_name_ + ".rotation_braking_accel").as_double());
  const double deadband = node->get_parameter(plugin_name_ + ".rotation_motor_deadband").as_double();
  const double margin = std::min(yaw_tolerance * 0.25,
    node->get_parameter(plugin_name_ + ".rotation_stop_margin").as_double());
  const double age = std::max(0., clock_->now().seconds() - rclcpp::Time(pose.header.stamp).seconds());
  if (!std::isfinite(age) || age > 0.5 || !std::isfinite(acceleration) || acceleration <= 0.) {
    throw nav2_core::PlannerException("Fresh physical heading is required for braking");
  }
  const double delay = node->get_parameter(plugin_name_ + ".rotation_command_delay").as_double() +
    control_duration_ + age;
  while (!angular_history_.empty() &&
    std::chrono::duration<double>(now - angular_history_.front().first).count() > delay)
  {
    angular_history_.pop_front();
  }
  double pending = 0.;
  for (const auto & sample : angular_history_) {
    pending = std::max(pending, std::abs(sample.second));
  }
  const double measured = have_yaw_rate_ ? measured_yaw_rate_ : 0.;
  const bool stopped = have_yaw_rate_ && std::abs(measured) <= 0.015 &&
    std::abs(last_angular_command_) <= 0.015 && pending <= 0.015;
  if (final_braking_ && stopped &&
    std::chrono::duration<double>(now - final_brake_started_).count() >= delay + control_duration_)
  {
    final_braking_ = false;
    if (++final_corrections_ > 12) {
      final_braking_failed_ = true;
      throw nav2_core::PlannerException("Final heading did not settle after bounded braking corrections");
    }
  }
  const double discriminant = acceleration * acceleration * delay * delay +
    2. * acceleration * std::max(0., remaining - margin);
  double target = std::min(wmax, std::max(0., std::sqrt(discriminant) - acceleration * delay));
  // Outside tolerance, allow command-space ramps to cross the physical INPUT
  // deadband. The following braking latch turns a tiny remaining angle into a
  // bounded pulse followed by an observed zero hold, rather than endless jitter.
  if (remaining > yaw_tolerance) {
    target = std::min(wmax, std::max(target, deadband + 0.01));
  }
  const double speed = std::abs(measured) +
    (std::abs(measured) > 0.005 ? acceleration * yaw_sample_interval_ / 2. : 0.);
  const double future_limit = std::max({speed, pending, target});
  const double accelerating = std::min(delay, std::max(0., future_limit - speed) / acceleration);
  const double projected = speed + acceleration * accelerating;
  const double stopping_angle = speed * accelerating + 0.5 * acceleration * accelerating * accelerating +
    projected * (delay - accelerating) + projected * projected / (2. * acceleration) + margin;
  const bool moving = std::abs(measured) > 0.015 || std::abs(last_angular_command_) > deadband;
  const bool wrong_direction = have_yaw_rate_ && measured * error < -1e-4;
  if (!final_braking_ && (remaining <= yaw_tolerance || wrong_direction ||
    (moving && remaining <= stopping_angle)))
  {
    final_braking_ = true;
    final_brake_started_ = now;
  }
  if (final_braking_) {
    target = 0.;
  }
  const double magnitude = std::abs(command.twist.angular.z);
  const double desired = std::min(magnitude, target);
  const double minimum = std::max(0., std::abs(last_angular_command_) - acceleration * step);
  // Native RPP has already applied the same ramp reference. Never increase its
  // checked angular speed; the normal production .8 acceleration preserves slew.
  const double limited = std::min(magnitude, std::max(desired, minimum));
  command.twist.angular.z = std::copysign(limited, command.twist.angular.z);
}

geometry_msgs::msg::TwistStamped RegulatedPathFollower::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(reference_mutex_);
  const auto now = Clock::now();
  const double elapsed = have_reference_ ?
    std::chrono::duration<double>(now - last_command_time_).count() : control_duration_;
  if (reset_requested_.exchange(false) || elapsed > 0.50 || elapsed < 0.) {
    reset_reference();
  }
  auto node = node_.lock();
  if (!node) {
    reset_reference();
    throw nav2_core::PlannerException("Navigation controller node disappeared");
  }
  // Only the angular command-space ramp reference is replaced. Real pose, TF,
  // linear odometry and RPP's complete native collision check remain unchanged.
  auto command_reference = velocity;
  command_reference.angular.z = have_reference_ ? last_angular_command_ : 0.;
  const double native_duration = control_duration_;
  // Native RPP uses control_duration_ for rotateToHeading acceleration. Do not
  // exceed the configured command slew when the controller executes early.
  control_duration_ = have_reference_ ?
    std::min(native_duration, std::max(0., elapsed)) : native_duration;
  observe_yaw(pose);
  const double command_step = control_duration_;
  try {
    if (final_braking_failed_) {
      throw nav2_core::PlannerException("Final heading braking correction budget exhausted");
    }
    auto command = Base::computeVelocityCommands(pose, command_reference, goal_checker);
    control_duration_ = native_duration;
    const double v = command.twist.linear.x;
    const double w = command.twist.angular.z;
    if (!std::isfinite(v) || !std::isfinite(w)) {
      throw nav2_core::PlannerException("Native RPP produced non-finite velocity");
    }
    // Match the final gate's limits without converting a tight arc into a
    // wider, unsafe one. Uniform slowdown traverses the same checked arc.
    const double vmax = node->get_parameter(plugin_name_ + ".max_linear_vel").as_double();
    const double wmax = node->get_parameter(plugin_name_ + ".max_angular_vel").as_double();
    const double scale = std::min({1., std::abs(v) > 0. ? vmax / std::abs(v) : 1.,
        std::abs(w) > 0. ? wmax / std::abs(w) : 1.});
    command.twist.linear.x *= scale;
    command.twist.angular.z *= scale;
    brake_final_rotation(command, pose, goal_checker, now, command_step);
    angular_history_.emplace_back(now, command.twist.angular.z);
    while (!angular_history_.empty() &&
      std::chrono::duration<double>(now - angular_history_.front().first).count() > 2.)
    {
      angular_history_.pop_front();
    }
    last_angular_command_ = command.twist.angular.z;
    last_command_time_ = now;
    have_reference_ = true;
    return command;
  } catch (...) {
    control_duration_ = native_duration;
    const bool final_failure = final_braking_failed_;
    reset_reference();
    // Repeated compute calls during Nav2 failure_tolerance must not restart a
    // spent correction budget. Only a new goal, Stop or lifecycle reset may.
    final_braking_failed_ = final_failure;
    throw;
  }
}

}  // namespace ee3033_navigation

PLUGINLIB_EXPORT_CLASS(ee3033_navigation::RegulatedPathFollower, nav2_core::Controller)
