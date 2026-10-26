cmake_minimum_required(VERSION 3.20)
project(dm_armx_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 一个库同时装"协议/串口/总线/关节"和"ros2_control 插件"。
# 插件必须编成 .so，controller_manager 才能在运行时 dlopen 它。
add_library(dm_armx_hardware SHARED
  src/dm_serial.cpp
  src/dm_frames.cpp
  src/dm_bus.cpp
  src/dm_joint.cpp
  src/dm_armx_hardware_interface.cpp
)

target_include_directories(dm_armx_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(dm_armx_hardware PUBLIC cxx_std_17)

target_link_libraries(dm_armx_hardware PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  hardware_interface::hardware_interface
  pluginlib::pluginlib
)

install(
  TARGETS dm_armx_hardware
  EXPORT export_dm_armx_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES dm_armx_hardware.xml DESTINATION share/${PROJECT_NAME})

# 让 pluginlib 登记这个插件，否则 controller_manager 找不到它
pluginlib_export_plugin_description_file(hardware_interface dm_armx_hardware.xml)

ament_export_targets(export_dm_armx_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)

ament_package()
