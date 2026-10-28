cmake_minimum_required(VERSION 3.8)
project(ee3033_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(ee3033_regulated_path_follower SHARED src/regulated_path_follower.cpp)
target_compile_features(ee3033_regulated_path_follower PUBLIC cxx_std_17)
target_compile_options(ee3033_regulated_path_follower PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(ee3033_regulated_path_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ee3033_regulated_path_follower
  nav2_core nav2_util nav2_regulated_pure_pursuit_controller rclcpp
  rclcpp_lifecycle pluginlib geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core regulated_path_follower.xml)

install(TARGETS ee3033_regulated_path_follower
  EXPORT export_ee3033_navigation
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(ee3033_regulated_path_follower)
ament_export_dependencies(nav2_core nav2_regulated_pure_pursuit_controller)
ament_package()
