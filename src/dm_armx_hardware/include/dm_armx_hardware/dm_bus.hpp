// dm_bus.hpp —— 一条总线上的收发与状态缓存（对应 Python 侧的 dm_bus.py）。
//
// 边界（与 dm_bus.py 一致）：
//   - **唯一发送出口是 `send_frame()`**：所有便利发送都走它，好计数、好替换；
//   - **不写寄存器** —— 它只发寄存器帧，写什么由调用方决定；
//   - **不自动使能、不判安全、不碰运动学、不拥有线程**（`wait_feedback`/`sync_states` 是阻塞的，
//     由调用方在合适的地方调用）。
//
// 两条真机教训已经内建在这里：
//   - 寄存器回包与反馈帧共用 `CMD=0x11` ⇒ 一律先过 `is_reg_response()`（陷阱 #13/#26），
//     否则寄存器回包会被当成反馈解出垃圾状态、污染缓存；
//   - 陈旧回包会骗缓存 ⇒ 要"排空 + 主动刷新到安静"时用 `sync_states()`，别自己写（陷阱 #20/#23）。

// 数据流（总线管理器，管理收发、缓存、注册）
// 发送
// 上层调 send_mit(id, ...)
//     ↓
// 调 mit_frame(...) 造帧（dm_frames）
//     ↓
// 调 send_frame(frame)  ← 唯一出口
//     ↓
// io_.write(字节)
// 接收
// io_.read_available(字节)
//     ↓
// rx_.feed(字节)
//     ↓
// rx_.drain() 切帧
//     ↓
// for 每帧：
//     is_reg_response?  → 进 reg_queue_
//     否则 → decode_feedback → 更新 motors_[id].state
//     ↓
// 返回解出的反馈帧数

#ifndef MOTOR_DRIVER_HARDWARE__DM_BUS_HPP_
#define MOTOR_DRIVER_HARDWARE__DM_BUS_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "dm_armx_hardware/dm_frames.hpp"
#include "dm_armx_hardware/dm_serial.hpp"

namespace dm_armx_hardware
{

struct MotorState
{
  uint8_t id = 0;
  uint8_t err = 0;
  double pos = 0.0;        // 电机侧 rad
  double vel = 0.0;
  double tau = 0.0;
  uint8_t t_mos_raw = 0;
  uint8_t t_rotor_raw = 0;
  bool valid = false;      // 收到过至少一条反馈才算有效
  double stamp = 0.0;      // steady_clock 秒
};

struct MotorEntry
{
  Limit limit{12.5, 10.0, 28.0};
  MotorState state;
  RxFrame last_frame{};    // 产生当前状态的那条原始帧（wait_feedback 要把它交出去）
};

class MotorBus
{
public:
  explicit MotorBus(SerialIo & io);
  ~MotorBus() = default;
  MotorBus(const MotorBus &) = delete;
  MotorBus & operator=(const MotorBus &) = delete;

  // ── 连接与注册 ──
  void open();                     // io.open() + 打开时清一次输入缓冲
  void close();
  bool is_open() const {return io_.is_open();}
  void add_motor(uint8_t motor_id, const Limit & limit);   // 重复注册 ⇒ 抛
  bool has_motor(uint8_t motor_id) const;
  void set_limit(uint8_t motor_id, const Limit & limit);
  const Limit & limit(uint8_t motor_id) const;             // 未注册 ⇒ 抛
  std::vector<uint8_t> registered_ids() const;

  // ── 发送（唯一出口）──
  void send_frame(const TxFrame & frame);
  void send_pos_vel(uint8_t motor_id, double p_des, double v_des);
  // 力位混控：0x300+id。i_des 是电流标幺（0..1），不是力矩指令。
  void send_force_pos(uint8_t motor_id, double p_des, double v_des, double i_des);
  // MIT：CAN ID 是 slave_id 本身（不是 0x100+id）。需要这台电机的档位来定标 ⇒ 未注册会抛。
  void send_mit(uint8_t motor_id, double p_des, double v_des, double kp, double kd, double t_ff);
  void send_enable(uint8_t motor_id);
  void send_disable(uint8_t motor_id);
  void send_refresh(uint8_t motor_id);

  // ── 接收 ──
  // 非阻塞抽干：把已经到的字节读进来、切帧、分类（反馈进缓存 / 寄存器回包进队列 / 陌生 ID 记账）。
  // 返回本次解出的**反馈帧**条数。
  std::size_t poll();
  // 阻塞等某一台电机的反馈（超时返回 false）。内部就是 poll + 0.5ms 睡眠。
  bool wait_feedback(uint8_t motor_id, RxFrame & out, double timeout_s);
  void flush();                    // 丢掉串口与内部缓冲里已有的字节
  std::optional<MotorState> get_state(uint8_t motor_id) const;
  const std::map<uint8_t, MotorEntry> & motors() const {return motors_;}
  std::vector<uint8_t> unknown_ids() const;

  // 排空陈旧回包 + 主动刷新，直到"安静 quiet_s"没有新帧（或超时）。
  // 想信状态之前用它 —— 自己写排空循环极易写错（陷阱 #23）。
  bool sync_states(double timeout_s = 2.0, double quiet_s = 0.05);

  // ── 寄存器 I/O（M5 切模式 / 写 PID 要用）──
  // 只发帧、不等反馈帧、不使能、不判安全；返回是否在超时内拿到匹配的回包。
  bool read_register(uint8_t motor_id, uint8_t rid, Word4 & out, double timeout_s = 0.05);
  bool write_register(uint8_t motor_id, uint8_t rid, const Word4 & raw4, double timeout_s = 0.05);
  bool save_params(uint8_t motor_id, double timeout_s = 0.5);

  const SerialStats & stats() const {return io_.stats();}

private:
  double now_s() const;
  // rid < 0 表示"任意 RID 都收"（存参数回包不一定回显 RID）
  bool wait_reg_response(uint8_t motor_id, uint8_t cmd, int rid, Word4 * out, double timeout_s);

  SerialIo & io_;
  RxBuf rx_;
  std::map<uint8_t, MotorEntry> motors_;
  std::set<uint8_t> unknown_ids_;
  std::vector<RxFrame> reg_queue_;   // 寄存器回包（等 read/write_register 来取）
};

}  // namespace dm_armx_hardware

#endif  // MOTOR_DRIVER_HARDWARE__DM_BUS_HPP_
