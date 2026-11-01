#include "dm_armx_hardware/dm_armx_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
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

bool param_bool(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, bool fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    return fallback;
  }
  return it->second == "true" || it->second == "True" || it->second == "1";
}

// <joint> 里的参数 → JointConfig。缺省值就是 xacro 里不写时的行为；
// motor_id 不给就按声明顺序 1..N。
JointConfig make_joint_config(
  const hardware_interface::ComponentInfo & info, std::size_t index, double default_vlim)
{
  JointConfig cfg;
  cfg.name = info.name;
  cfg.motor_id = static_cast<uint8_t>(param_double(info.parameters, "motor_id", index + 1.0));
  cfg.direction = static_cast<int>(param_double(info.parameters, "direction", 1.0));
  cfg.offset = param_double(info.parameters, "offset", 0.0);
  cfg.scale = param_double(info.parameters, "scale", 1.0);
  cfg.mode = static_cast<ControlMode>(
    static_cast<int>(param_double(info.parameters, "mode", 2.0)));
  cfg.kp = param_double(info.parameters, "kp", 0.0);
  cfg.kd = param_double(info.parameters, "kd", 0.0);
  cfg.velocity_limit = param_double(info.parameters, "velocity_limit", default_vlim);
  cfg.grip_force_default = param_double(info.parameters, "grip_force_default", 0.0);
  cfg.grip_force_max = param_double(info.parameters, "grip_force_max", 0.0);
  cfg.current_per_force = param_double(info.parameters, "current_per_force", 0.0);
  cfg.fatal_on_error = param_bool(info.parameters, "fatal_on_error", true);
  cfg.limit = Limit{
    param_double(info.parameters, "p_max", 12.5),
    param_double(info.parameters, "v_max", 30.0),
    param_double(info.parameters, "t_max", 10.0)};

  // 软限位在 xacro 里是"可选"的：没写就是 NaN → 不钳。六轴就不钳，夹爪才钳。
  const double pmin = param_double(info.parameters, "position_min", std::nan(""));
  const double pmax = param_double(info.parameters, "position_max", std::nan(""));
  if (std::isfinite(pmin)) {cfg.position_min = pmin;}
  if (std::isfinite(pmax)) {cfg.position_max = pmax;}
  return cfg;
}

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

  serial_ = std::make_unique<SerialPort>(port, baud);
  bus_ = std::make_unique<MotorBus>(*serial_);

  joints_.clear();
  joints_.reserve(info.joints.size());
  for (std::size_t i = 0; i < info.joints.size(); ++i) {
    const JointConfig cfg = make_joint_config(info.joints[i], i, velocity_limit_);
    joints_.push_back(std::make_unique<Joint>(*bus_, cfg));
    RCLCPP_INFO(
      get_logger(), "关节 %s -> 电机 0x%02X（direction=%d, offset=%.3f, scale=%.3f, mode=%d）",
      cfg.name.c_str(), cfg.motor_id, cfg.direction, cfg.offset, cfg.scale,
      static_cast<int>(cfg.mode));
  }

  RCLCPP_INFO(
    get_logger(), "串口 %s @ %d bps，关节数 %zu，默认限速 %.2f rad/s",
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

  // 按 xacro 的 mode 写 CTRL_MODE(0x0A)：模式不对时控制帧会被静默忽略。
  // 写参数会让驱动复位，所以先失能；写完回读确认。手臂必须已有支撑。
  if (!apply_control_modes()) {
    return CallbackReturn::ERROR;
  }

  // 先拿到实测位置：位置类模式下"没有实测位置就使能"会让电机朝零位冲
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

  // 位置类帧只在 CTRL_MODE 对得上时才被电机执行：2 → 0x100+ID，4 → 0x300+ID。
  // 模式不对时，使能、读反馈都正常，但控制帧被**静默忽略** —— 表现为"控制器说成功，电机不动"。
  if (!check_control_modes()) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "串口已打开，%zu 台电机都有反馈", joints_.size());

  // 总线上还有别的电机在回帧（比如刚接上来的夹爪）：
  // 打印它们的反馈 ID，方便确认 ID 有没有和手臂的 1..6 / 0x11..0x16 撞车。
  for (uint8_t unknown_id : bus_->unknown_ids()) {
    RCLCPP_WARN(
      get_logger(),
      "总线上还有未注册的电机反馈 ID 0x%02X（夹爪？确认它的 ESC_ID/MST_ID 与手臂不冲突）",
      unknown_id);
  }
  return CallbackReturn::SUCCESS;
}

bool DmArmxHardwareInterface::apply_control_modes()
{
  // 只做一件事：把每个关节的 CTRL_MODE(0x0A) 拉到 xacro 的 mode。
  // 写参数会让驱动器复位（pvt_motor_test.py 已实测），所以：
  //   1) 写之前先失能 —— 会卸掉该关节的力矩，手臂必须已有支撑；
  //   2) 写完等 0.2 s 再回读；回读不符就 fail fast，不带着错模式使能。
  for (const auto & joint : joints_) {
    const uint8_t id = joint->motor_id();
    const auto & cfg = joint->config();
    const uint32_t expected = static_cast<uint32_t>(cfg.mode);

    Word4 raw{};
    if (bus_->read_register(id, 0x0A, raw, 0.2)) {
      const uint32_t mode = uint8s_to_uint32(raw.data());
      if (mode == expected) {
        RCLCPP_INFO(
          get_logger(), "关节 %s：CTRL_MODE 已是 %u，跳过", cfg.name.c_str(), mode);
        continue;
      }
      RCLCPP_WARN(
        get_logger(), "关节 %s：CTRL_MODE=%u → %u，先失能再写",
        cfg.name.c_str(), mode, expected);
    } else {
      RCLCPP_WARN(
        get_logger(), "关节 %s：读 CTRL_MODE 超时，直接写 %u", cfg.name.c_str(), expected);
    }

    joint->disable();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 写 ack 可能在驱动复位时丢掉；以回读为准。
    if (!bus_->write_register(id, 0x0A, uint32_to_uint8s(expected), 0.3)) {
      RCLCPP_WARN(
        get_logger(), "关节 %s：写 CTRL_MODE 的 ack 没等到，靠回读确认", cfg.name.c_str());
    }

    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      Word4 rb{};
      ok = bus_->read_register(id, 0x0A, rb, 0.3) &&
        uint8s_to_uint32(rb.data()) == expected;
    }
    if (!ok) {
      RCLCPP_ERROR(
        get_logger(), "关节 %s：写 CTRL_MODE=%u 后回读不符，拒绝启动",
        cfg.name.c_str(), expected);
      return false;
    }
    RCLCPP_INFO(get_logger(), "关节 %s：CTRL_MODE 已写成 %u", cfg.name.c_str(), expected);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  return true;
}

bool DmArmxHardwareInterface::check_control_modes()
{
  for (const auto & joint : joints_) {
    Word4 raw{};
    if (!bus_->read_register(joint->motor_id(), 0x0A, raw, 0.1)) {
      RCLCPP_WARN(
        get_logger(), "关节 %s：读 CTRL_MODE(0x0A) 超时，跳过模式检查",
        joint->name().c_str());
      continue;
    }
    const uint32_t expected = static_cast<uint32_t>(joint->config().mode);
    const uint32_t mode = uint8s_to_uint32(raw.data());
    if (mode == expected) {
      RCLCPP_INFO(
        get_logger(), "关节 %s：CTRL_MODE=%u（%s）", joint->name().c_str(), mode,
        mode == 1 ? "MIT" : (mode == 2 ? "位置速度" : "力位混控"));
      continue;
    }
    if (mode >= 1 && mode <= 4) {
      RCLCPP_ERROR(
        get_logger(),
        "关节 %s（电机 0x%02X）CTRL_MODE=%u，期望 %u；"
        "控制帧会被电机忽略。请用达妙上位机把 0x0A 改成 %u 并保存，再重新上电",
        joint->name().c_str(), joint->motor_id(), mode, expected, expected);
      return false;
    }
    RCLCPP_WARN(
      get_logger(), "关节 %s：CTRL_MODE 回读异常(%u)，跳过模式检查",
      joint->name().c_str(), mode);
  }
  return true;
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
        get_logger(), "启动位置：%s = %+.4f（关节侧单位，电机 0x%02X）",
        joint->name().c_str(), joint->get_state().position, joint->motor_id());
      joint->enable();           // 使能 + 立刻发一帧"保持当前实测位置"

      // 命令初值 = 实测位置：控制器还没发命令时，write() 也不会朝 0 冲
      set_command(
        joint->name() + "/" + hardware_interface::HW_IF_POSITION,
        joint->get_state().position);
      if (joint->config().mode == ControlMode::kForcePos) {
        // 力控制器没起时 effort 命令就用默认力；0 也会被 Joint 映射成默认力
        set_command(
          joint->name() + "/" + hardware_interface::HW_IF_EFFORT,
          joint->config().grip_force_default);
      } else {
        // 六轴（MIT/位置）的 effort 初值 0：MIT 使能后先发 0 力矩，等 controller 接管
        set_command(joint->name() + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
      }
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

  return CallbackReturn::SUCCESS;
}

DmArmxHardwareInterface::CallbackReturn DmArmxHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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
    if (joint->config().mode == ControlMode::kForcePos ||
      joint->config().mode == ControlMode::kMit)
    {
      set_state(prefix + hardware_interface::HW_IF_EFFORT, st.torque);
    }

    if (st.err != 0x0 && st.err != 0x1) {
      const char * text = err_text(st.err);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "关节 %s 报故障 ERR=0x%X（%s）",
        joint->name().c_str(), st.err, text != nullptr ? text : "未知错误码");
      // 夹爪故障（fatal_on_error=false）只报警，不让整臂失能
      if (joint->config().fatal_on_error) {
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DmArmxHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & joint : joints_) {
    const double target =
      get_command(joint->name() + "/" + hardware_interface::HW_IF_POSITION);
    try {
      if (joint->config().mode == ControlMode::kMit) {
        double tau = get_command(joint->name() + "/" + hardware_interface::HW_IF_EFFORT);
        if (!std::isfinite(tau)) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "%s 的 MIT 力矩命令是 NaN/inf，按 0 发", joint->name().c_str());
          tau = 0.0;
        }
        // p_des 用实测位置（kp=0 时无关紧要）；v_des=0；kd 由 xacro 给。
        joint->set_mit(
          joint->get_state().position, 0.0, joint->config().kp, joint->config().kd, tau);
      } else if (joint->config().mode == ControlMode::kForcePos) {
        const double force =
          get_command(joint->name() + "/" + hardware_interface::HW_IF_EFFORT);
        joint->set_force_pos(target, joint->config().velocity_limit, force);
      } else {
        joint->set_pos_vel(target, joint->config().velocity_limit);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "下发 %s 失败：%s", joint->name().c_str(), e.what());
      if (joint->config().fatal_on_error) {
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace dm_armx_hardware

PLUGINLIB_EXPORT_CLASS(
  dm_armx_hardware::DmArmxHardwareInterface, hardware_interface::SystemInterface)
