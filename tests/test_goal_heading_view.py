"""Exercise the real GoalChecker delegate against the installed ROS interface."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]


def test_goal_heading_delegate_preserves_real_checker_semantics(tmp_path):
    ros_include = Path('/opt/ros/humble/include')
    if not ros_include.is_dir():
        pytest.skip('Run this semantic delegate check in the native Humble environment')
    compiler = shutil.which('g++') or shutil.which('c++')
    assert compiler
    source = tmp_path / 'heading_view_check.cpp'
    source.write_text(r'''
#include <cassert>
#include <limits>
#include "ee3033_navigation/goal_heading_view.hpp"
class Checker final : public nav2_core::GoalChecker {
public:
  int resets=0, initialized=0, checks=0;
  bool valid=true;
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {++initialized;}
  void reset() override {++resets;}
  bool isGoalReached(const geometry_msgs::msg::Pose & query,
    const geometry_msgs::msg::Pose & goal, const geometry_msgs::msg::Twist & velocity) override {
    ++checks; return query.position.x==goal.position.x && velocity.linear.x==0.;
  }
  bool getTolerances(geometry_msgs::msg::Pose & p, geometry_msgs::msg::Twist & v) override {
    p.position.x=.12; p.position.y=.12; p.orientation.w=.42; v.angular.z=.03; return valid;
  }
};
int main() {
  Checker real;
  ee3033_navigation::GoalHeadingView far(real,1.5), near(real,.10);
  geometry_msgs::msg::Pose p; geometry_msgs::msg::Twist v;
  assert(far.getTolerances(p,v));
  assert(p.position.x==0. && p.position.y==0.);
  assert(p.orientation.w==.42 && v.angular.z==.03);
  assert(real.getTolerances(p,v) && p.position.x==.12 && p.position.y==.12);
  assert(near.getTolerances(p,v) && p.position.x==.12 && p.position.y==.12);
  assert(real.resets==0 && real.initialized==0 && real.checks==0);
  geometry_msgs::msg::Pose query,goal; query.position.x=1.; goal.position.x=2.;
  assert(!far.isGoalReached(query,goal,v));
  goal.position.x=1.; assert(far.isGoalReached(query,goal,v));
  assert(real.checks==2);
  far.reset(); assert(real.resets==1);
  rclcpp_lifecycle::LifecycleNode::WeakPtr node;
  far.initialize(node,"test",nullptr); assert(real.initialized==1);
  real.valid=false;
  bool rejected=false;
  try {far.getTolerances(p,v);} catch(const nav2_core::PlannerException &) {rejected=true;}
  assert(rejected);
  real.valid=true;
  ee3033_navigation::GoalHeadingView invalid(real,std::numeric_limits<double>::quiet_NaN());
  rejected=false;
  try {invalid.getTolerances(p,v);} catch(const nav2_core::PlannerException &) {rejected=true;}
  assert(rejected);
}
''')
    binary = tmp_path / 'heading_view_check'
    includes = [ros_include, *(p for p in ros_include.iterdir() if p.is_dir()),
        ROOT / 'src/ee3033_navigation/include']
    command = [compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror']
    for include in includes:
        command += ['-I', str(include)]
    subprocess.run([*command, str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
