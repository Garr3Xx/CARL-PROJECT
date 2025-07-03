cmake_minimum_required(VERSION 3.16)
project(carl_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(carl_control_node src/carl_control_node.cpp)
add_executable(manual_teleop src/manual_teleop.cpp)
add_executable(autostop_node src/autostop_node.cpp)
add_executable(odom_node src/odom_node.cpp)

ament_target_dependencies(carl_control_node rclcpp geometry_msgs std_msgs control_msgs)
ament_target_dependencies(manual_teleop rclcpp std_msgs geometry_msgs)
ament_target_dependencies(autostop_node rclcpp geometry_msgs sensor_msgs std_msgs control_msgs)
ament_target_dependencies(odom_node 
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  carl_control_node
  manual_teleop
  autostop_node
  odom_node
  DESTINATION lib/${PROJECT_NAME}

)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
