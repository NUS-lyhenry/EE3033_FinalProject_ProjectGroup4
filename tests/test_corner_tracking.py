"""Compile the controller's geometry-only selector without ROS or simulation."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]


def test_native_corner_selector_handles_later_corners_stairs_cusps_and_straight_reset(tmp_path):
    compiler = shutil.which('c++') or shutil.which('clang++') or shutil.which('g++')
    if compiler is None:
        pytest.skip('A C++ compiler is required for native corner-selector checks')
    source = tmp_path / 'corner_checks.cpp'
    source.write_text(r'''
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>
#include "ee3033_navigation/corner_tracking.hpp"
#include "ee3033_navigation/goal_heading_trigger.hpp"
#include "ee3033_navigation/raw_cost_clearance.hpp"
using ee3033_navigation::PathPoint;
using ee3033_navigation::has_sharp_bend;
int main() {
  assert(!has_sharp_bend({}));
  assert(!has_sharp_bend({{0, 0}, {0, 0}, {0, 0}}));
  assert(!has_sharp_bend({{0, 0}, {1, 0}, {2, 0}}));
  // This bend is later than a local35cm preview; no replan is assumed.
  assert(has_sharp_bend({{0,0}, {1,0}, {2,0}, {2,1}}));
  const auto features=ee3033_navigation::sharp_bend_features({{0,0}, {1,0}, {2,0}, {2,1}});
  assert(!ee3033_navigation::near_sharp_bend(features,{0,0}));
  assert(ee3033_navigation::near_sharp_bend(features,{1.8,0}));
  assert(!ee3033_navigation::near_sharp_bend(features,{2,.5}));
  // The SAME cached, unreplanned path supports all three physical positions.
  std::vector<PathPoint> transformed_features;
  for(const auto & p:features) transformed_features.push_back({1.2-p.y,-.4+p.x});
  assert(ee3033_navigation::near_sharp_bend(transformed_features,{1.2,1.4}));
  assert(!ee3033_navigation::near_sharp_bend(transformed_features,{1.2,-.4}));
  assert(has_sharp_bend({{2,1}, {2,0}, {1,0}, {0,0}}));
  assert(has_sharp_bend({{0,0}, {.2,0}, {0,0}}));
  assert(has_sharp_bend({{0,0}, {.2,0}, {.2,.2}}));
  // A straight replacement has no sticky state from the previous L plan.
  assert(!has_sharp_bend({{2,0}, {2,.5}, {2,1}}));
  std::vector<PathPoint> stairs;
  for (int i=0; i<80; ++i) {
    stairs.push_back({i*.0125, i*.0125});
    stairs.push_back({(i+1)*.0125, i*.0125});
  }
  assert(!has_sharp_bend(stairs));
  std::vector<PathPoint> jitter;
  for (int i=0; i<80; ++i) jitter.push_back({i*.025, (i%2 ? .00625 : -.00625)});
  assert(!has_sharp_bend(jitter));
  std::vector<PathPoint> broad_curve, tight_curve;
  for (int i=0; i<=60; ++i) {
    const double angle=i*1.5707963267948966/60.;
    broad_curve.push_back({.5*std::sin(angle), .5*(1-std::cos(angle))});
    tight_curve.push_back({.1*std::sin(angle), .1*(1-std::cos(angle))});
  }
  assert(!has_sharp_bend(broad_curve));
  assert(has_sharp_bend(tight_curve));
  assert(has_sharp_bend({{0,0}, {std::numeric_limits<double>::quiet_NaN(),0}, {1,0}}));
  using ee3033_navigation::actual_goal_within_xy_tolerance;
  assert(!actual_goal_within_xy_tolerance(1.5,.12));
  assert(!actual_goal_within_xy_tolerance(.120001,.12));
  assert(actual_goal_within_xy_tolerance(.12,.12));
  assert(actual_goal_within_xy_tolerance(.024,.025));
  assert(!actual_goal_within_xy_tolerance(.026,.025));
  assert(actual_goal_within_xy_tolerance(.059,.06));
  bool rejected=false;
  try {actual_goal_within_xy_tolerance(1.,0.);} catch(const std::invalid_argument &) {rejected=true;}
  assert(rejected);
  rejected=false;
  try {ee3033_navigation::near_sharp_bend(features,{std::numeric_limits<double>::quiet_NaN(),0});}
  catch(const std::invalid_argument &) {rejected=true;}
  assert(rejected);
  std::vector<unsigned char> raw(21*21,0);
  using ee3033_navigation::raw_obstacle_near;
  assert(!raw_obstacle_near(raw.data(),21,21,.05,0,0,.5,.5));
  raw[10*21+14]=253; // Inflation is not a raw physical obstacle.
  assert(!raw_obstacle_near(raw.data(),21,21,.05,0,0,.5,.5));
  raw[10*21+14]=254;
  assert(raw_obstacle_near(raw.data(),21,21,.05,0,0,.5,.5));
  raw[10*21+14]=255;
  assert(raw_obstacle_near(raw.data(),21,21,.05,0,0,.5,.5));
  assert(raw_obstacle_near(raw.data(),21,21,.05,3.,-2.,3.5,-1.5));
  raw[10*21+14]=0; raw[10*21+15]=254;
  assert(!raw_obstacle_near(raw.data(),21,21,.05,0,0,.5,.5));
  raw[10*21+15]=0; raw[14*21+14]=254; // Bounding-box overlap is insufficient.
  assert(!raw_obstacle_near(raw.data(),21,21,.05,0,0,.5,.5));
  assert(raw_obstacle_near(raw.data(),21,21,.05,0,0,.1,.1)); // Outside is unknown.
  std::vector<unsigned char> boundary(8*8,0);
  boundary[4*8+2]=254; // Cell ends exactly at the lower x-radius boundary.
  assert(raw_obstacle_near(boundary.data(),8,8,.25,0,0,1.,1.,.25));
  boundary[4*8+2]=0; boundary[2*8+4]=254;
  assert(raw_obstacle_near(boundary.data(),8,8,.25,0,0,1.,1.,.25));
  boundary[2*8+4]=0; boundary[2*8+2]=254;
  assert(!raw_obstacle_near(boundary.data(),8,8,.25,0,0,1.,1.,.25));
  std::vector<unsigned char> decimal(21*21,0);
  decimal[10*21+14]=254; // .48+.22 rounds just below the next .05 boundary.
  assert(raw_obstacle_near(decimal.data(),21,21,.05,0,0,.48,.50,.22));
  decimal[10*21+14]=0; decimal[14*21+10]=255;
  assert(raw_obstacle_near(decimal.data(),21,21,.05,0,0,.50,.48,.22));
  decimal[14*21+10]=0; decimal[14*21+14]=254;
  assert(!raw_obstacle_near(decimal.data(),21,21,.05,0,0,.48,.48,.22));
}
''')
    binary = tmp_path / 'corner_checks'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
        '-I', str(ROOT / 'src/ee3033_navigation/include'), str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
