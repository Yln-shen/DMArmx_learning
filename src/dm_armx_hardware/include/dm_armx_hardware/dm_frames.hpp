// dm_frames.hpp —— 达妙电机**协议层**（帧编解码纯函数）。
//
// 这一层是什么：
//   30 字节适配器发送帧的构造（MIT / 位置速度 / 力位混控 / 使能失能 / 刷新 / 寄存器读·写·存参）、
//   16 字节接收帧的切分与解码（反馈帧 / 寄存器回包）。
//
// 这一层**不是**什么（边界，改之前先读）：
//   - **不开发送**：只返回字节，串口在 dm_serial（M4）；
//   - **不碰串口 / 不 sleep / 不打屏**（官方例程 damiao.h 里有 std::cout 与 usleep，这里都不要）；
//   - **不判安全**：不钳软限位、不看 ERR、不决定使能 —— 那是 joint/arm 那一层的活；
//   - **不碰运动学**。
//
// ⚠️ 这是 Python 侧 `motor_driver/dm_frames.py` 的**第二份实现**。两份必须逐字节一致，
//    靠 `test/test_dm_frames.cpp` 的**对拍测试**保证（C++ 造帧 vs 直接调 Python 造帧）。
//    改这里或改 Python 那份，都要跑 `colcon test --packages-select dm_armx_hardware`。
//
// 改编自达妙官方 `C++例程/u2can/include/damiao.h`（已 vendored 到 src/third_party/C++例程/），
// 去掉了打屏/usleep/param_map，并按 Python 侧的语义补齐了寄存器帧与回包分类。

// 数据流（切帧拆帧造帧，“协议层”——负责“参数 ↔ 字节”的双向转换）
// 发送方向
// 上层调 mit_frame(参数)
//     ↓
// 定点映射（浮点 → 整数）
//     ↓
// 拼 8 字节数据段
//     ↓
// 拼 30 字节发送帧
//     ↓
// 返回 TxFrame
// 接收方向
// 串口收到字节流
//     ↓
// RxBuf.feed(字节)
//     ↓
// RxBuf.drain() 切出 16 字节帧
//     ↓
// is_reg_response() 判断类型
//     ↓
// decode_feedback() 或 decode_reg_response()
//     ↓
// 返回结构体

#ifndef MOTOR_DRIVER_HARDWARE__DM_FRAMES_HPP_
#define MOTOR_DRIVER_HARDWARE__DM_FRAMES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace dm_armx_hardware
{

// ── 帧长与命令字（与 dm_frames.py 一一对应）──
constexpr std::size_t kTxFrameLen = 30;   // 主机→适配器
constexpr std::size_t kRxFrameLen = 16;   // 适配器→主机
constexpr std::size_t kDataLen = 8;       // 数据段

constexpr uint8_t kFeedbackCmd = 0x11;    // 反馈帧 CMD
constexpr uint8_t kCmdEnable = 0xFC;
constexpr uint8_t kCmdDisable = 0xFD;
constexpr uint16_t kCanIdBroadcast = 0x7FF;   // 刷新帧与寄存器帧（目标 ID 在数据段里）
constexpr uint16_t kCanIdPosVelBase = 0x100;  // 位置速度 = 0x100 + slave_id
constexpr uint16_t kCanIdForcePosBase = 0x300;  // 力位混控 = 0x300 + slave_id
// MIT 用的是 **slave_id 本身**（不是 0x100+id）—— 别搞混

constexpr uint8_t kRegCmdRead = 0x33;
constexpr uint8_t kRegCmdWrite = 0x55;
constexpr uint8_t kRegCmdSave = 0xAA;

using TxFrame = std::array<uint8_t, kTxFrameLen>;
using RxFrame = std::array<uint8_t, kRxFrameLen>;
using Data8 = std::array<uint8_t, kDataLen>;
using Word4 = std::array<uint8_t, 4>;

// ── 每台电机自己的映射范围，来自寄存器 0x15/0x16/0x17 的**回读值** ──
//   ⚠️ 4340P 与 4310 档位不同（实测 4340P: 12.5/10/28；4310: 12.5/30/10），**用错档位解出来的速度和力矩差数倍**。
struct Limit
{
  double p_max;
  double v_max;
  double t_max;
};

// ── 定点映射原语（与厂商 `float_to_uint` 逐字节一致）──
//   先钳位到 [x_min, x_max]，再线性映射到 [0, 2^bits - 1]；**向零截断**，不是四舍五入。
double limit_min_max(double x, double x_min, double x_max);
uint32_t float_to_uint(double x, double x_min, double x_max, int bits);

// ── 4 字节小端 ──
Word4 float32_to_uint8s(double value);       // float32 小端（位置速度 / 力位混控用）
Word4 uint32_to_uint8s(uint32_t value);      // uint32 小端（寄存器写用）
uint32_t uint8s_to_uint32(const uint8_t * b4);
double uint8s_to_float32(const uint8_t * b4);

// ── 发送帧构造 ──
TxFrame build_tx(uint16_t can_id, const Data8 & data);
TxFrame mit_frame(uint8_t slave_id, double p_des, double v_des, double kp, double kd,
                  double t_ff, const Limit & limit);
TxFrame pos_vel_frame(uint8_t slave_id, double p_des, double v_des);
TxFrame force_pos_frame(uint8_t slave_id, double p_des, double v_des, double i_des);
TxFrame cmd_frame(uint8_t slave_id, uint8_t cmd);   // 只接受 0xFC / 0xFD，其它抛异常
TxFrame refresh_frame(uint8_t slave_id);
TxFrame reg_read_frame(uint8_t slave_id, uint8_t rid);
TxFrame reg_write_frame(uint8_t slave_id, uint8_t rid, const Word4 & raw4);
TxFrame save_params_frame(uint8_t slave_id);

// ── 接收帧切分 ──
//   extract_rx：一次性切（尾部残片会丢，只在"已知是完整一批"时用）
//   RxBuf：带残留的缓冲（**收串口必须用它**，残片留到下一次，绝不丢）
std::vector<RxFrame> extract_rx(const uint8_t * buf, std::size_t len);
std::vector<RxFrame> extract_rx(const std::vector<uint8_t> & buf);

class RxBuf
{
public:
  void feed(const uint8_t * chunk, std::size_t len);
  void feed(const std::vector<uint8_t> & chunk);
  std::vector<RxFrame> drain();          // 切出所有完整帧，保留尾部残片
  std::size_t pending() const {return buf_.size();}
  void clear() {buf_.clear();}

private:
  std::vector<uint8_t> buf_;
};

// ── 解码 ──
struct Feedback
{
  uint8_t id;          // D[0] 低 4 位
  uint8_t err;         // D[0] 高 4 位（0 失能 / 1 使能 / 其余故障；0xD 通讯丢失锁存）
  double pos;          // rad（电机侧）
  double vel;          // rad/s
  double tau;          // N·m
  uint8_t t_mos_raw;   // D[6]（℃ 量级，原样给上层）
  uint8_t t_rotor_raw; // D[7]
  Data8 raw;
};

// ⚠️ `limit` 必须来自该电机自己的 0x15/0x16/0x17 回读值，不能全局写死
Feedback decode_feedback(const uint8_t * data8, const Limit & limit);
Feedback decode_feedback(const Data8 & data8, const Limit & limit);

const char * err_text(uint8_t err);

// 这条 16 字节帧是**寄存器回包**而不是控制反馈帧吗？
//
// ⚠️ 判据里 `frame[7] <= 0x0F && frame[8] == 0x00` 是 2026-10-03 修出来的（Python 侧陷阱 #26）：
//    反馈帧的 `D[2]` 就是 POS16 的**低字节**，只看 `D[2] ∈ {0x33,0x55,0xAA}` 会把停在
//    特定位置的电机反馈**全部丢掉**（那台电机在缓存里彻底隐身）。真寄存器回包的
//    `D[0:2]` 恒为"目标电机 ID 小端"（`D[0] ≤ 0x0F`、`D[1] == 0x00`），补上即可分开。
bool is_reg_response(const RxFrame & frame);

struct RegResponse
{
  uint16_t motor_id;
  uint8_t cmd;         // 0x33 读 / 0x55 写 / 0xAA 存参数
  uint8_t rid;
  Word4 data;          // 低位在 data[0]
  Data8 raw;
};
RegResponse decode_reg_response(const RxFrame & frame);

}  // namespace dm_armx_hardware

#endif  // MOTOR_DRIVER_HARDWARE__DM_FRAMES_HPP_
