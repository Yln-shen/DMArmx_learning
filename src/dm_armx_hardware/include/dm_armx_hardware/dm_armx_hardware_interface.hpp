#ifndef DM_ARMX_HARDWARE__DM_ARMX_HARDWARE_INTERFACE_HPP_
#define DM_ARMX_HARDWARE__DM_ARMX_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dm_armx_hardware/dm_bus.hpp"
#include "dm_armx_hardware/dm_joint.hpp"
#include "dm_armx_hardware/dm_serial.hpp"

namespace dm_armx_hardware
{

// 最小可动的 ros2_control 硬件插件：
//   只暴露 position 命令 + position/velocity 状态，只走"位置速度(POS_VEL)"模式。
//   生命周期：on_init(读参数) → on_configure(开串口/拿反馈) → on_activate(使能/锁住当前位置)
//            → 100 Hz read()/write() → on_deactivate(失能)
class DmArmxHardwareInterface : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 声明顺序 = 构造顺序：serial_ 必须先于 bus_ 存在（MotorBus 持有 SerialIo &）
  std::unique_ptr<SerialPort> serial_;
  std::unique_ptr<MotorBus> bus_;
  std::vector<std::unique_ptr<Joint>> joints_;

  double velocity_limit_ = 0.5;   // rad/s，第一版所有关节统一限速
  // // false = 标定模式：只读反馈、不使能、不下发，方便用手把真机摆到 URDF 零位
  // bool enable_on_activate_ = true;
};

}  // namespace dm_armx_hardware

#endif  // DM_ARMX_HARDWARE__DM_ARMX_HARDWARE_INTERFACE_HPP_
