#include "dm_armx_hardware/dm_armx_hardware_interface.hpp"

#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace dm_armx_hardware
{

namespace
{

std::string param_string(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return fallback;
  }
  return it->second;
}

double param_double(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, double fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return fallback;
  }
  return std::stod(it->second);
}

// bool param_bool(
//   const std::unordered_map<std::string, std::string> & params,
//   const std::string & key, bool fallback)
// {
//   const auto it = params.find(key);
//   if (it == params.end() || it->second.empty()) {
//     return fallback;
//   }
//   return it->second == "true" || it->second == "True" || it->second == "1";
// }

}  // namespace

DmArmxHardwareInterface::CallbackReturn DmArmxHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;
  const std::string port = param_string(info.hardware_parameters, "serial_port", "/dev/ttyACM0");
  const int baud = static_cast<int>(param_double(info.hardware_parameters, "baud_rate", 921600.0));
  velocity_limit_ = param_double(info.hardware_parameters, "velocity_limit", 0.5);
  // enable_on_activate_ = param_bool(info.hardware_parameters, "enable_on_activate", true);

  serial_ = std::make_unique<SerialPort>(port, baud);
  bus_ = std::make_unique<MotorBus>(*serial_);

  joints_.clear();
  joints_.reserve(info.joints.size());
  for (std::size_t i = 0; i < info.joints.size(); ++i) {
    const auto & joint_info = info.joints[i];
    JointConfig cfg;
    cfg.name = joint_info.name;
    cfg.motor_id = static_cast<uint8_t>(
      param_double(joint_info.parameters, "motor_id", static_cast<double>(i + 1)));
    cfg.direction = static_cast<int>(param_double(joint_info.parameters, "direction", 1.0));
    cfg.offset = param_double(joint_info.parameters, "offset", 0.0);
    cfg.limit = Limit{
      param_double(joint_info.parameters, "p_max", 12.5),
      param_double(joint_info.parameters, "v_max", 30.0),
      param_double(joint_info.parameters, "t_max", 10.0)};

    joints_.push_back(std::make_unique<Joint>(*bus_, cfg));
    RCLCPP_INFO(
      get_logger(), "关节 %s -> 电机 0x%02X（direction=%d, offset=%.3f）",
      cfg.name.c_str(), cfg.motor_id, cfg.direction, cfg.offset);
  }

  RCLCPP_INFO(
    get_logger(), "串口 %s @ %d bps，关节数 %zu，限速 %.2f rad/s",
    port.c_str(), baud, joints_.size(), velocity_limit_);
  return CallbackReturn::SUCCESS;
}

DmArmxHardwareInterface::CallbackReturn DmArmxHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    bus_->open();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "打开串口失败：%s", e.what());
    return CallbackReturn::ERROR;
  }

  // 先拿到实测位置：位置速度模式下"没有实测位置就使能"会让电机朝零位冲
  if (!bus_->sync_states(2.0, 0.05)) {
    RCLCPP_ERROR(get_logger(), "等待电机反馈超时");
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : joints_) {
    if (!joint->has_state()) {
      RCLCPP_ERROR(
        get_logger(), "关节 %s（电机 0x%02X）没有任何反馈：查 ID / 波特率 / 电源 / 控制模式",
        joint->name().c_str(), joint->motor_id());
      return CallbackReturn::ERROR;
    }
  }

  // 位置速度帧（CAN ID = 0x100+ID）只有在 CTRL_MODE=2 时才会被电机执行。
  // 模式不对时，使能、读反馈都正常，但位置指令被**静默忽略** —— 表现为"控制器说成功，电机不动"。
  for (const auto & joint : joints_) {
    Word4 raw{};
    if (!bus_->read_register(joint->motor_id(), 0x0A, raw, 0.1)) {
      RCLCPP_WARN(
        get_logger(), "关节 %s：读 CTRL_MODE(0x0A) 超时，跳过模式检查",
        joint->name().c_str());
      continue;
    }
    const uint32_t mode = uint8s_to_uint32(raw.data());
    if (mode == 2) {
      RCLCPP_INFO(get_logger(), "关节 %s：CTRL_MODE=2（位置速度）", joint->name().c_str());
      continue;
    }
    if (mode >= 1 && mode <= 4) {
      RCLCPP_ERROR(
        get_logger(),
        "关节 %s（电机 0x%02X）CTRL_MODE=%u，不是位置速度(2)；"
        "位置帧会被电机忽略。请用达妙上位机把 0x0A 改成 2 并保存，再重新上电",
        joint->name().c_str(), joint->motor_id(), mode);
      return CallbackReturn::ERROR;
    }
    RCLCPP_WARN(
      get_logger(), "关节 %s：CTRL_MODE 回读异常(%u)，跳过模式检查",
      joint->name().c_str(), mode);
  }

  RCLCPP_INFO(get_logger(), "串口已打开，%zu 台电机都有反馈", joints_.size());
  return CallbackReturn::SUCCESS;
}

DmArmxHardwareInterface::CallbackReturn DmArmxHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    // 使能前再刷一次：避免用几秒前的旧位置当"保持帧"
    bus_->sync_states(1.0, 0.03);

    for (const auto & joint : joints_) {
      joint->assert_healthy();   // ERR 不是 0/1 ⇒ 抛异常，并且已经先失能
      // 把启动时每个关节的实测位置打出来：第一次上电要靠它判断方向/零位对不对
      RCLCPP_INFO(
        get_logger(), "启动位置：%s = %+.3f rad（电机 0x%02X）",
        joint->name().c_str(), joint->get_state().position, joint->motor_id());
      joint->enable();           // 使能 + 立刻发一帧"保持当前实测位置"
        // if (enable_on_activate_) {
      //   joint->enable();         // 使能 + 立刻发一帧"保持当前实测位置"
      // }
    }

    // 命令初值 = 实测位置：控制器还没发命令时，write() 也不会朝 0 冲
    for (const auto & joint : joints_) {
      set_command(
        joint->name() + "/" + hardware_interface::HW_IF_POSITION,
        joint->get_state().position);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "使能失败：%s", e.what());
    for (const auto & joint : joints_) {
      try {
        joint->disable();
      } catch (const std::exception &) {
      }
    }
    return CallbackReturn::ERROR;
  }

  // if (!enable_on_activate_) {
  //   RCLCPP_WARN(
  //     get_logger(),
  //     "标定模式（enable_on_activate=false）：电机保持失能，可以手动摆动机械臂；"
  //     "摆到 URDF 零位后读 /joint_states，用 raw = offset + direction*model 反算新 offset");
  // } else {
  //   RCLCPP_INFO(get_logger(), "已使能 %zu 台电机，命令锁定在当前位置", joints_.size());
  // }
  return CallbackReturn::SUCCESS;
}

DmArmxHardwareInterface::CallbackReturn DmArmxHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // if (!enable_on_activate_) {
  //   return CallbackReturn::SUCCESS;   // 标定模式本来就没使能
  // }
  for (const auto & joint : joints_) {
    try {
      joint->disable();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "关节 %s 失能失败：%s", joint->name().c_str(), e.what());
    }
  }
  RCLCPP_INFO(get_logger(), "已失能 %zu 台电机", joints_.size());
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DmArmxHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bus_->poll();

  for (const auto & joint : joints_) {
    if (!joint->has_state()) {
      continue;   // 还没收到过这一台的反馈：保持上一次的状态
    }
    const JointState st = joint->get_state();
    const std::string prefix = joint->name() + "/";
    set_state(prefix + hardware_interface::HW_IF_POSITION, st.position);
    set_state(prefix + hardware_interface::HW_IF_VELOCITY, st.velocity);

    if (st.err != 0x0 && st.err != 0x1) {
      const char * text = err_text(st.err);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "关节 %s 报故障 ERR=0x%X（%s）",
        joint->name().c_str(), st.err, text != nullptr ? text : "未知错误码");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DmArmxHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // if (!enable_on_activate_) {
  //   return hardware_interface::return_type::OK;   // 标定模式：一帧都不发
  // }
  for (const auto & joint : joints_) {
    const double target =
      get_command(joint->name() + "/" + hardware_interface::HW_IF_POSITION);
    try {
      joint->set_pos_vel(target, velocity_limit_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "下发 %s 失败：%s", joint->name().c_str(), e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace dm_armx_hardware

PLUGINLIB_EXPORT_CLASS(
  dm_armx_hardware::DmArmxHardwareInterface, hardware_interface::SystemInterface)
