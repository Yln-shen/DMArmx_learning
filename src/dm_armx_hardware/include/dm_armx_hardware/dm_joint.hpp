// dm_joint.hpp —— 单个关节（语义对齐 Python 的 joint.py）。
//
// 边界：
//   - 不拥有控制循环（`set_*` 只发一帧）、不 poll、不写寄存器；
//   - 不知道 ROS、不读 yaml —— 参数由调用方（插件）填进 `JointConfig`；
//   - **只做 mode 2(位置速度)**。MIT(1) 已从 C++ 这一层砍掉（2026-10-14 决定：本链只走
//     POS_VEL，固件位置环自带积分、不需要力矩前馈；折叠位的重力由机械结构承载，已实测）。
//     ⚠️ 协议层的 `dm_frames::mit_frame` **保留**（Python 侧 `joint.py` 仍然支持 MIT，
//        两份协议实现要继续互相咬住），只是 C++ 的控制路径不再用它。
//
// 三条真机教训内建在这里：
//   - 位置类模式下**没有实测位置就拒绝使能**（切模式时电机内部指令被清零，使能会朝零位冲）；
//   - 使能后**立刻补"保持帧"**，且保持帧用**电机侧实测值**、不经过软限位/换算（钳了就不是保持）；
//   - `prepare_frame()` 的顺序固定为 NaN 拦 → 软限位 → 换算 → PMAX 钳位。
#ifndef MOTOR_DRIVER_HARDWARE__DM_JOINT_HPP_
#define MOTOR_DRIVER_HARDWARE__DM_JOINT_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "dm_armx_hardware/dm_bus.hpp"
#include "dm_armx_hardware/dm_frames.hpp"

namespace dm_armx_hardware
{

constexpr double kHoldVlim = 0.1;   // 保持帧的速度幅值（与 Python 侧 HOLD_VLIM 一致）

// 模型坐标（URDF / ros2_control 接口用的那一套） ⇄ 我们的关节坐标 q_ours。
//   q_urdf = sign · q_ours + zero_shift        （sign/zero_shift 来自 arm_description/config/align.yaml）
// 放在这里是为了让**插件与测试用同一份公式**，而不是各写一遍。
inline double model_to_ours(double q_urdf, int sign, double zero_shift)
{
  return static_cast<double>(sign) * (q_urdf - zero_shift);
}

inline double ours_to_model(double q_ours, int sign, double zero_shift)
{
  return static_cast<double>(sign) * q_ours + zero_shift;
}

struct JointConfig
{
  std::string name;
  uint8_t motor_id = 1;
  std::string motor_type;                       // "4340P" / "4310"（只用于报错信息）
  int direction = 1;                            // ±1：电机侧 = direction × 关节侧 + offset
  double offset = 0.0;
  Limit limit{12.5, 10.0, 28.0};                // 档位（必须来自 0x15/0x16/0x17 回读）
  std::optional<double> position_min;           // 关节侧软限位（我们的标定）；空 = 不钳
  std::optional<double> position_max;
};

struct JointState
{
  std::string name;
  uint8_t motor_id = 0;
  uint8_t err = 0;
  double position = 0.0;      // 关节侧（已换算）
  double velocity = 0.0;      // 关节侧（只乘 direction）
  double torque = 0.0;        // 关节侧（只乘 direction）
  uint8_t t_mos_raw = 0;
  uint8_t t_rotor_raw = 0;
  bool enabled = false;
  double stamp = 0.0;
};

class Joint
{
public:
  // 构造时会**校验/注册**到 bus（与 Python 侧一致）：总线上一台电机只能有一个映射范围，
  // 否则解出来的力矩差数倍且不报错。
  Joint(MotorBus & bus, const JointConfig & cfg);
  ~Joint() = default;
  Joint(const Joint &) = delete;
  Joint & operator=(const Joint &) = delete;

  const JointConfig & config() const {return cfg_;}
  const std::string & name() const {return cfg_.name;}
  uint8_t motor_id() const {return cfg_.motor_id;}

  // ── 换算与钳位（纯函数，和 Python 的公式逐条对应）──
  double joint_to_motor(double joint_value) const;   // direction × v + offset
  double motor_to_joint(double motor_value) const;   // direction × (v − offset)
  double clamp(double joint_value) const;            // 软限位（NaN 不在这里拦，见 prepare_frame）
  double clamp_pmax(double motor_value) const;       // 电机侧 ±PMAX

  // 发送前的统一处理：NaN 拦 → 软限位 → 换算 → PMAX。返回**电机侧**位置。
  double prepare_frame(double joint_pos) const;

  // ── 发送（每条只发一帧）──
  void set_pos_vel(double joint_pos, double vlim);
  void enable();     // 命令帧 + 保持帧（用电机侧实测位置，绝不朝零位冲）
  void disable();

  // ── 状态 ──
  bool has_state() const;
  JointState get_state() const;      // 没有反馈 ⇒ 抛（"从没收到过"不该被当成 0）
  void assert_healthy() const;       // 只查 ERR；故障 ⇒ 先失能再抛

private:
  MotorBus & bus_;
  JointConfig cfg_;
};

}  // namespace dm_armx_hardware

#endif  // MOTOR_DRIVER_HARDWARE__DM_JOINT_HPP_
