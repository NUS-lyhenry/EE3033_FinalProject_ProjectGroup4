"""Compile the actual controller parameter validator with minimal ROS value stubs."""
from pathlib import Path
import shutil
import subprocess
import pytest


def test_native_controller_accepts_tracking_choices_and_rejects_invalid_ros_writes(tmp_path):
    compiler=shutil.which('c++')
    if not compiler:
        pytest.skip('A C++ compiler is required')
    root=Path(__file__).parents[1]
    cpp=(root/'src/ee3033_navigation/src/regulated_path_follower.cpp').read_text()
    start=cpp.index('rcl_interfaces::msg::SetParametersResult RegulatedPathFollower::validate_caps(')
    end=cpp.index('\nvoid RegulatedPathFollower::reset_reference()',start)
    source=tmp_path/'bounds.cpp'
    source.write_text(r'''
#include <cassert>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
namespace rcl_interfaces { namespace msg {
struct SetParametersResult {bool successful=false;std::string reason;};
}}
namespace rclcpp {
enum class ParameterType {PARAMETER_DOUBLE,PARAMETER_BOOL};
struct Parameter {
 std::string name;double value;ParameterType type;
 Parameter(std::string n,double v):name(n),value(v),type(ParameterType::PARAMETER_DOUBLE){}
 Parameter(std::string n,bool v):name(n),value(v),type(ParameterType::PARAMETER_BOOL){}
 const std::string &get_name()const{return name;}
 ParameterType get_type()const{return type;}
 double as_double()const{return value;}
};
}
class RegulatedPathFollower {
 std::string plugin_name_="FollowPath";
 public:rcl_interfaces::msg::SetParametersResult validate_caps(const std::vector<rclcpp::Parameter>&)const;
};
'''+cpp[start:end]+r'''
int main(){
 RegulatedPathFollower controller;
 for(double distance:{.06,.08,.18,.30})for(double angle:{.05,.10,.35,1.57}) {
  const auto result=controller.validate_caps({{"FollowPath.lookahead_dist",distance},
    {"FollowPath.min_lookahead_dist",distance},{"FollowPath.max_lookahead_dist",distance},
    {"FollowPath.rotate_to_heading_min_angle",angle}});
  assert(result.successful);
 }
 for(const auto &name:{"lookahead_dist","min_lookahead_dist","max_lookahead_dist","rotate_to_heading_min_angle"}) {
  std::string key="FollowPath.";key+=name;
  for(double bad:{0.,-.1,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
   assert(!controller.validate_caps({{key,bad}}).successful);
  assert(!controller.validate_caps({{key,true}}).successful);
 }
 assert(!controller.validate_caps({{"FollowPath.lookahead_dist",.31}}).successful);
 assert(!controller.validate_caps({{"FollowPath.rotate_to_heading_min_angle",1.58}}).successful);
 // Reject the complete atomic request when any tracking field is invalid.
 assert(!controller.validate_caps({{"FollowPath.max_linear_vel",.16},
   {"FollowPath.lookahead_dist",.18},{"FollowPath.rotate_to_heading_min_angle",.01}}).successful);
 assert(controller.validate_caps({{"FollowPath.max_linear_vel",.16},
   {"FollowPath.rotation_braking_accel",.8},{"FollowPath.rotation_motor_deadband",.08}}).successful);
 assert(!controller.validate_caps({{"FollowPath.rotation_braking_accel",.9}}).successful);
}
''')
    exe=tmp_path/'bounds'
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',str(source),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
