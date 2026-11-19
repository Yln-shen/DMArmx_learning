cmake_minimum_required(VERSION 3.20)
project(usb_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

# ---- 可执行文件 ----
add_executable(usb_camera_node src/usb_camera_node.cpp)

target_include_directories(usb_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(usb_camera_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  cv_bridge::cv_bridge
  camera_info_manager::camera_info_manager
  ${OpenCV_LIBS}
)

install(
  TARGETS usb_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
