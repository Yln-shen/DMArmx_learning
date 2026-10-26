// dm_bus.cpp —— 总线层实现（与 dm_frames 的边界配合：它只搬字节与缓存状态）。
#include "dm_armx_hardware/dm_bus.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

namespace dm_armx_hardware
{

namespace
{
constexpr double kPollSleepS = 0.0005;          // 500us：921600 下一帧只要 0.17ms
constexpr std::size_t kRegQueueMax = 32;        // 没人取的寄存器回包别无限堆
}  // namespace

MotorBus::MotorBus(SerialIo & io)
: io_(io)
{
}

double MotorBus::now_s() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ── 连接与注册 ──
void MotorBus::open()
{
  io_.open();
  rx_.clear();
  reg_queue_.clear();
  unknown_ids_.clear();
  io_.flush_input();      // 打开时设备里可能已经堆了陈旧字节
}

void MotorBus::close()
{
  io_.close();
}

void MotorBus::add_motor(uint8_t motor_id, const Limit & limit)
{
  if (motors_.count(motor_id) != 0) {
    throw std::invalid_argument("电机 " + std::to_string(motor_id) + " 已经注册过了");
  }
  MotorEntry e;
  e.limit = limit;
  motors_.emplace(motor_id, e);
}

bool MotorBus::has_motor(uint8_t motor_id) const
{
  return motors_.count(motor_id) != 0;
}

void MotorBus::set_limit(uint8_t motor_id, const Limit & limit)
{
  auto it = motors_.find(motor_id);
  if (it == motors_.end()) {
    throw std::invalid_argument("电机 " + std::to_string(motor_id) + " 没注册，不能改档位");
  }
  it->second.limit = limit;
}

const Limit & MotorBus::limit(uint8_t motor_id) const
{
  auto it = motors_.find(motor_id);
  if (it == motors_.end()) {
    throw std::invalid_argument("电机 " + std::to_string(motor_id) + " 没注册，读不到档位");
  }
  return it->second.limit;
}

std::vector<uint8_t> MotorBus::registered_ids() const
{
  std::vector<uint8_t> ids;
  ids.reserve(motors_.size());
  for (const auto & kv : motors_) {ids.push_back(kv.first);}
  return ids;
}

// ── 发送 ──
void MotorBus::send_frame(const TxFrame & frame)
{
  if (!io_.is_open()) {throw std::runtime_error("总线没开（open() 了吗）");}
  io_.write(frame.data(), frame.size());
}

void MotorBus::send_pos_vel(uint8_t motor_id, double p_des, double v_des)
{
  send_frame(pos_vel_frame(motor_id, p_des, v_des));
}

void MotorBus::send_mit(uint8_t motor_id, double p_des, double v_des, double kp, double kd,
  double t_ff)
{
  // mit_frame 要用这台电机自己的档位（p_max/v_max/t_max）⇒ limit() 顺便强制"必须已注册"：
  // 用错档位解出来的力矩差数倍且不报错（陷阱 #9）。
  send_frame(mit_frame(motor_id, p_des, v_des, kp, kd, t_ff, limit(motor_id)));
}

void MotorBus::send_enable(uint8_t motor_id)
{
  send_frame(cmd_frame(motor_id, kCmdEnable));
}

void MotorBus::send_disable(uint8_t motor_id)
{
  send_frame(cmd_frame(motor_id, kCmdDisable));
}

void MotorBus::send_refresh(uint8_t motor_id)
{
  send_frame(refresh_frame(motor_id));
}

// ── 接收 ──
std::size_t MotorBus::poll()
{
  if (!io_.is_open()) {return 0;}
  std::vector<uint8_t> chunk;
  io_.read_available(chunk);
  if (!chunk.empty()) {rx_.feed(chunk);}

  std::size_t feedback_count = 0;
  for (const RxFrame & f : rx_.drain()) {
    if (is_reg_response(f)) {                  // 先分流：寄存器回包**绝不能**当反馈解
      reg_queue_.push_back(f);
      if (reg_queue_.size() > kRegQueueMax) {
        reg_queue_.erase(reg_queue_.begin());
      }
      continue;
    }
    if (f[1] != kFeedbackCmd) {continue;}       // 不是反馈帧（未知 CMD）：忽略
    const uint8_t motor_id = static_cast<uint8_t>(f[7] & 0x0F);
    auto it = motors_.find(motor_id);
    if (it == motors_.end()) {                 // 陌生 ID：记账，不污染缓存
      unknown_ids_.insert(motor_id);
      continue;
    }
    const Feedback fb = decode_feedback(f.data() + 7, it->second.limit);
    MotorState & st = it->second.state;
    st.id = fb.id;
    st.err = fb.err;
    st.pos = fb.pos;
    st.vel = fb.vel;
    st.tau = fb.tau;
    st.t_mos_raw = fb.t_mos_raw;
    st.t_rotor_raw = fb.t_rotor_raw;
    st.valid = true;
    st.stamp = now_s();
    it->second.last_frame = f;      // 留着给 wait_feedback 交出去
    feedback_count += 1;
  }
  return feedback_count;
}

bool MotorBus::wait_feedback(uint8_t motor_id, RxFrame & out, double timeout_s)
{
  auto it = motors_.find(motor_id);
  if (it == motors_.end()) {
    throw std::invalid_argument("电机 " + std::to_string(motor_id) + " 没注册，等不到它的反馈");
  }
  // 只认"这次调用之后新到的那一帧"：记下当前时间戳，等它变大
  const double stamp_before = it->second.state.stamp;
  const double deadline = now_s() + timeout_s;
  while (true) {
    poll();
    if (it->second.state.valid && it->second.state.stamp > stamp_before) {
      out = it->second.last_frame;
      return true;
    }
    if (now_s() >= deadline) {return false;}
    std::this_thread::sleep_for(std::chrono::duration<double>(kPollSleepS));
  }
}

void MotorBus::flush()
{
  if (io_.is_open()) {io_.flush_input();}
  rx_.clear();
}

std::optional<MotorState> MotorBus::get_state(uint8_t motor_id) const
{
  auto it = motors_.find(motor_id);
  if (it == motors_.end() || !it->second.state.valid) {return std::nullopt;}
  return it->second.state;
}

std::vector<uint8_t> MotorBus::unknown_ids() const
{
  return std::vector<uint8_t>(unknown_ids_.begin(), unknown_ids_.end());
}

bool MotorBus::sync_states(double timeout_s, double quiet_s)
{
  flush();
  for (const auto & kv : motors_) {send_refresh(kv.first);}
  const double start = now_s();
  double last_new = start;
  while (true) {
    const std::size_t n = poll();
    const double now = now_s();
    if (n > 0) {last_new = now;}
    if (now - last_new >= quiet_s) {return true;}      // 安静了 ⇒ 缓存已是真相
    if (now - start >= timeout_s) {return false;}
    std::this_thread::sleep_for(std::chrono::duration<double>(kPollSleepS));
  }
}

// ── 寄存器 I/O ──
bool MotorBus::wait_reg_response(uint8_t motor_id, uint8_t cmd, int rid, Word4 * out,
  double timeout_s)
{
  const double deadline = now_s() + timeout_s;
  while (true) {
    poll();
    for (auto it = reg_queue_.begin(); it != reg_queue_.end(); ++it) {
      const RegResponse r = decode_reg_response(*it);
      const bool rid_ok = (rid < 0) || (r.rid == static_cast<uint8_t>(rid));
      if (r.motor_id == motor_id && r.cmd == cmd && rid_ok) {
        if (out != nullptr) {*out = r.data;}
        reg_queue_.erase(it);
        return true;
      }
    }
    if (now_s() >= deadline) {return false;}
    std::this_thread::sleep_for(std::chrono::duration<double>(kPollSleepS));
  }
}

bool MotorBus::read_register(uint8_t motor_id, uint8_t rid, Word4 & out, double timeout_s)
{
  reg_queue_.clear();                    // 只认"本条命令"的应答（= Python 侧的 flush 语义）
  send_frame(reg_read_frame(motor_id, rid));
  return wait_reg_response(motor_id, kRegCmdRead, rid, &out, timeout_s);
}

bool MotorBus::write_register(uint8_t motor_id, uint8_t rid, const Word4 & raw4,
  double timeout_s)
{
  reg_queue_.clear();
  send_frame(reg_write_frame(motor_id, rid, raw4));
  return wait_reg_response(motor_id, kRegCmdWrite, rid, nullptr, timeout_s);
}

bool MotorBus::save_params(uint8_t motor_id, double timeout_s)
{
  reg_queue_.clear();
  send_frame(save_params_frame(motor_id));
  // 存参数的回包不一定回显哪个 RID ⇒ 这里不挑 RID（rid < 0）
  return wait_reg_response(motor_id, kRegCmdSave, -1, nullptr, timeout_s);
}

}  // namespace dm_armx_hardware
