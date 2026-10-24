cmake_minimum_required(VERSION 3.20)
project(dm_armx_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(dm_armx_interfaces REQUIRED)


add_executable(moveit src/moveit.cpp)

add_executable(commander_template src/commander_template.cpp)

target_link_libraries(moveit
  rclcpp::rclcpp
  moveit_ros_planning_interface::moveit_ros_planning_interface
)

target_link_libraries(commander_template PUBLIC
  rclcpp::rclcpp
  moveit_ros_planning_interface::moveit_ros_planning_interface
  example_interfaces::example_interfaces
  geometry_msgs::geometry_msgs
  moveit_msgs::moveit_msgs
  tf2::tf2
  dm_armx_interfaces::dm_armx_interfaces
)

install(
  TARGETS moveit
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    moveit
    commander_template
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
