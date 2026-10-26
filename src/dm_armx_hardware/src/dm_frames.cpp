// dm_frames.cpp —— 协议层实现（与 Python 侧 motor_driver/dm_frames.py 逐字节对齐）。
#include "dm_armx_hardware/dm_frames.hpp"

#include <algorithm>
#include <cstring>

namespace dm_armx_hardware
{

namespace
{

// 发送帧模板（30 字节）。实测回读：
//     55 aa 1e 03 01 00 00 00 0a 00 | 00 00 00 00 00 00 00 00 08 00 | 00 ... 00
//   [0:2]=55 AA 帧头  [2]=0x1E=30 帧长  [8]=0x0A  [18]=0x08，其余固定 0。
// ⚠️ 不可推导的魔数：改错任何一个字节 = 适配器不认帧 = 电机不动，而且**不会有任何报错**。
const TxFrame kTxTemplate = {
  0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// 寄存器帧的数据段头 4 字节：[ID_L, ID_H, CMD, RID]
Data8 reg_head(uint8_t slave_id, uint8_t cmd, uint8_t rid = 0)
{
  return {static_cast<uint8_t>(slave_id & 0xFF), static_cast<uint8_t>((slave_id >> 8) & 0xFF),
    cmd, rid, 0, 0, 0, 0};
}

}  // namespace

// ───────────────────────── 定点映射 ─────────────────────────
double limit_min_max(double x, double x_min, double x_max)
{
  // 与 SDK 的 LIMIT_MIN_MAX 一致：x == x_min 判进下界、x == x_max 判进上界
  if (x <= x_min) {return x_min;}
  if (x > x_max) {return x_max;}
  return x;
}

uint32_t float_to_uint(double x, double x_min, double x_max, int bits)
{
  const double clamped = limit_min_max(x, x_min, x_max);
  // ⚠️ 运算顺序与 Python 完全一致（先减、再除、后乘），否则浮点结果可能差 1 个 LSB；
  //    截断用 C++ 的强制转换（向零截断），与 Python 的 int() 一致。
  return static_cast<uint32_t>((clamped - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

Word4 float32_to_uint8s(double value)
{
  const float f = static_cast<float>(value);
  Word4 out{};
  std::memcpy(out.data(), &f, sizeof(float));   // 目标平台是小端（x86_64 / aarch64）
  return out;
}

Word4 uint32_to_uint8s(uint32_t value)
{
  return {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>((value >> 16) & 0xFF), static_cast<uint8_t>((value >> 24) & 0xFF)};
}

uint32_t uint8s_to_uint32(const uint8_t * b4)
{
  return static_cast<uint32_t>(b4[0]) | (static_cast<uint32_t>(b4[1]) << 8) |
         (static_cast<uint32_t>(b4[2]) << 16) | (static_cast<uint32_t>(b4[3]) << 24);
}

double uint8s_to_float32(const uint8_t * b4)
{
  const uint32_t bits = uint8s_to_uint32(b4);
  float f = 0.0F;
  std::memcpy(&f, &bits, sizeof(float));
  return static_cast<double>(f);
}

// ───────────────────────── 发送帧 ─────────────────────────
TxFrame build_tx(uint16_t can_id, const Data8 & data)
{
  TxFrame frame = kTxTemplate;
  frame[13] = static_cast<uint8_t>(can_id & 0xFF);
  frame[14] = static_cast<uint8_t>((can_id >> 8) & 0xFF);
  std::copy(data.begin(), data.end(), frame.begin() + 21);
  return frame;
}

TxFrame mit_frame(uint8_t slave_id, double p_des, double v_des, double kp, double kd,
  double t_ff, const Limit & limit)
{
  // 位布局与厂商 controlMIT 逐位一致；Kp/Kd 是**线性映射**（Kp∈[0,500]、Kd∈[0,5]）
  const uint32_t q_u = float_to_uint(p_des, -limit.p_max, limit.p_max, 16);
  const uint32_t dq_u = float_to_uint(v_des, -limit.v_max, limit.v_max, 12);
  const uint32_t kp_u = float_to_uint(kp, 0.0, 500.0, 12);
  const uint32_t kd_u = float_to_uint(kd, 0.0, 5.0, 12);
  const uint32_t t_u = float_to_uint(t_ff, -limit.t_max, limit.t_max, 12);
  const Data8 data = {
    static_cast<uint8_t>((q_u >> 8) & 0xFF), static_cast<uint8_t>(q_u & 0xFF),
    static_cast<uint8_t>((dq_u >> 4) & 0xFF),
    static_cast<uint8_t>(((dq_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)),
    static_cast<uint8_t>(kp_u & 0xFF),
    static_cast<uint8_t>((kd_u >> 4) & 0xFF),
    static_cast<uint8_t>(((kd_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F)),
    static_cast<uint8_t>(t_u & 0xFF),
  };
  return build_tx(slave_id, data);   // CAN ID = slave_id 本身（不是 0x100+id）
}

TxFrame pos_vel_frame(uint8_t slave_id, double p_des, double v_des)
{
  // 数据 = float32(p) + float32(v)，**无缩放**（位置单位直接是输出轴 rad）
  const Word4 p = float32_to_uint8s(p_des);
  const Word4 v = float32_to_uint8s(v_des);
  const Data8 data = {p[0], p[1], p[2], p[3], v[0], v[1], v[2], v[3]};
  return build_tx(static_cast<uint16_t>(kCanIdPosVelBase + slave_id), data);
}

TxFrame force_pos_frame(uint8_t slave_id, double p_des, double v_des, double i_des)
{
  // D[0:4]=float32(p)  D[4:6]=uint16(v*100)  D[6:8]=uint16(i*10000)
  // ⚠️ 两个 uint16 都是无符号：负值钳到 0、超上限静默钳到 10000（与 float_to_uint 的钳位行为一致）
  const double v_capped = std::max(0.0, std::min(v_des, 100.0));
  const double i_capped = std::max(0.0, std::min(i_des, 1.0));
  const uint16_t v_u = static_cast<uint16_t>(v_capped * 100.0);
  const uint16_t i_u = static_cast<uint16_t>(i_capped * 10000.0);
  const Word4 p = float32_to_uint8s(p_des);
  const Data8 data = {p[0], p[1], p[2], p[3],
    static_cast<uint8_t>(v_u & 0xFF), static_cast<uint8_t>((v_u >> 8) & 0xFF),
    static_cast<uint8_t>(i_u & 0xFF), static_cast<uint8_t>((i_u >> 8) & 0xFF)};
  return build_tx(static_cast<uint16_t>(kCanIdForcePosBase + slave_id), data);
}

TxFrame cmd_frame(uint8_t slave_id, uint8_t cmd)
{
  if (cmd != kCmdEnable && cmd != kCmdDisable) {
    throw std::invalid_argument("未知命令：只支持 0xFC 使能 / 0xFD 失能");
  }
  const Data8 data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
  return build_tx(slave_id, data);
}

TxFrame refresh_frame(uint8_t slave_id)
{
  const Data8 data = {static_cast<uint8_t>(slave_id & 0xFF),
    static_cast<uint8_t>((slave_id >> 8) & 0xFF), 0xCC, 0, 0, 0, 0, 0};
  return build_tx(kCanIdBroadcast, data);
}

TxFrame reg_read_frame(uint8_t slave_id, uint8_t rid)
{
  return build_tx(kCanIdBroadcast, reg_head(slave_id, kRegCmdRead, rid));
}

TxFrame reg_write_frame(uint8_t slave_id, uint8_t rid, const Word4 & raw4)
{
  const Data8 head = reg_head(slave_id, kRegCmdWrite, rid);
  const Data8 data = {head[0], head[1], head[2], head[3], raw4[0], raw4[1], raw4[2], raw4[3]};
  return build_tx(kCanIdBroadcast, data);
}

TxFrame save_params_frame(uint8_t slave_id)
{
  // ⚠️ D[3] = 0x01 是**按手册**（SDK 的 save_motor_param 发 0x00）—— 真机断电验证过手册是对的
  return build_tx(kCanIdBroadcast, reg_head(slave_id, kRegCmdSave, 0x01));
}

// ───────────────────────── 接收帧切分 ─────────────────────────
std::vector<RxFrame> extract_rx(const uint8_t * buf, std::size_t len)
{
  std::vector<RxFrame> frames;
  std::size_t i = 0;
  while (i + kRxFrameLen <= len) {
    if (buf[i] == 0xAA && buf[i + kRxFrameLen - 1] == 0x55) {
      RxFrame f{};
      std::copy(buf + i, buf + i + kRxFrameLen, f.begin());
      frames.push_back(f);
      i += kRxFrameLen;
    } else {
      i += 1;
    }
  }
  return frames;
}

std::vector<RxFrame> extract_rx(const std::vector<uint8_t> & buf)
{
  return extract_rx(buf.data(), buf.size());
}

void RxBuf::feed(const uint8_t * chunk, std::size_t len)
{
  if (len > 0) {buf_.insert(buf_.end(), chunk, chunk + len);}
}

void RxBuf::feed(const std::vector<uint8_t> & chunk)
{
  feed(chunk.data(), chunk.size());
}

std::vector<RxFrame> RxBuf::drain()
{
  std::vector<RxFrame> out;
  std::size_t i = 0;
  while (i + kRxFrameLen <= buf_.size()) {
    if (buf_[i] == 0xAA && buf_[i + kRxFrameLen - 1] == 0x55) {
      RxFrame f{};
      std::copy(buf_.begin() + static_cast<std::ptrdiff_t>(i),
        buf_.begin() + static_cast<std::ptrdiff_t>(i + kRxFrameLen), f.begin());
      out.push_back(f);
      i += kRxFrameLen;
    } else {
      i += 1;
    }
  }
  // 残片**留着**：任何"读一次切一次"都会让下一帧错位（见 dm_frames.py 的同名注释）
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(i));
  return out;
}

// ───────────────────────── 解码 ─────────────────────────
namespace
{
inline double u_to_float(uint32_t u, int bits, double lo, double hi)
{
  return (u / static_cast<double>((1u << bits) - 1)) * (hi - lo) + lo;
}
}  // namespace

Feedback decode_feedback(const uint8_t * data8, const Limit & limit)
{
  Feedback fb{};
  fb.err = static_cast<uint8_t>((data8[0] >> 4) & 0x0F);
  fb.id = static_cast<uint8_t>(data8[0] & 0x0F);
  const uint32_t pos_u = (static_cast<uint32_t>(data8[1]) << 8) | data8[2];
  const uint32_t vel_u = (static_cast<uint32_t>(data8[3]) << 4) | (data8[4] >> 4);
  const uint32_t tau_u = (static_cast<uint32_t>(data8[4] & 0x0F) << 8) | data8[5];
  fb.pos = u_to_float(pos_u, 16, -limit.p_max, limit.p_max);
  fb.vel = u_to_float(vel_u, 12, -limit.v_max, limit.v_max);
  fb.tau = u_to_float(tau_u, 12, -limit.t_max, limit.t_max);
  fb.t_mos_raw = data8[6];
  fb.t_rotor_raw = data8[7];
  std::copy(data8, data8 + kDataLen, fb.raw.begin());
  return fb;
}

Feedback decode_feedback(const Data8 & data8, const Limit & limit)
{
  return decode_feedback(data8.data(), limit);
}

const char * err_text(uint8_t err)
{
  switch (err) {
    case 0x0: return "失能（未使能 / 已失能）";
    case 0x1: return "使能";
    case 0x8: return "超压";
    case 0x9: return "欠压";
    case 0xA: return "过电流";
    case 0xB: return "MOS 过温";
    case 0xC: return "电机线圈过温";
    case 0xD: return "通讯丢失（超时，锁存：只能断电清）";
    case 0xE: return "过载";
    default: return nullptr;   // 未知码：Python 侧返回 "未知错误码 N"，这里交给调用方决定怎么显示
  }
}

bool is_reg_response(const RxFrame & frame)
{
  // ⚠️ 后两个条件（D[0] ≤ 0x0F 且 D[1] == 0x00）是修出来的：反馈帧的 D[2] 是 POS16 低字节，
  //    只按 D[2] 判会把停在特定位置的电机反馈全部误丢（Python 侧陷阱 #26）。
  const bool cmd_ok = frame[9] == kRegCmdRead || frame[9] == kRegCmdWrite ||
    frame[9] == kRegCmdSave;
  return frame[1] == kFeedbackCmd && cmd_ok && frame[7] <= 0x0F && frame[8] == 0x00;
}

RegResponse decode_reg_response(const RxFrame & frame)
{
  const uint8_t * d = frame.data() + 7;
  RegResponse out{};
  out.motor_id = static_cast<uint16_t>((d[1] << 8) | d[0]);
  out.cmd = d[2];
  out.rid = d[3];
  out.data = {d[4], d[5], d[6], d[7]};
  std::copy(d, d + kDataLen, out.raw.begin());
  return out;
}

}  // namespace dm_armx_hardware
