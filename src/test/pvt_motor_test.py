#!/usr/bin/env python3
"""pvt_motor_test.py —— 达妙电机 PVT（力位混控，CTRL_MODE=4）单机测试脚本。

## 先把名词对齐（模式选错一定调不通）

说明书「力位混控模式」一节的调试步骤是：把电机模式切成 **PVT**，再在调试页选「位置」子页。
所以这里的 PVT == 力位混控 == CTRL_MODE 4，控制帧 CAN ID = **0x300 + ID**：

    D[0:4]  float32  p_des          目标位置，rad（输出轴），**无缩放**
    D[4:6]  uint16   v_des × 100    限速，rad/s，无符号，0..10000 → 0..100 rad/s
    D[6:8]  uint16   i_des × 10000  电流上限标幺，无符号，0..10000 → 0..1.0

和位置速度模式（0x100+ID，只有 p/v）只差最后那个 i_des：位置环照常走，但输出电流被
i_des 卡住。夹爪就是靠它把「夹多紧」变成「最多出多大力」。

## 为什么不能照抄 MIT 例程

- 模式不对时 0x300 帧会被**静默忽略**：使能正常、反馈正常、电机不动，一个报错都没有；
- i_des / v_des 是**无符号**：写负数会被静默钳成 0（= 一点力都不出）；
- 位置类模式下**没有实测位置就使能**：驱动器内部指令被清零，电机会朝零位冲。

## 脚本做了什么

1. 等反馈 → 读 0x0A 确认 CTRL_MODE=4（不对就提示 `--set-mode 4`）；
2. 读 0x15/0x16/0x17 的 PMAX/VMAX/TMAX 回读值，和命令行档位对一下
   （档位错 = 反馈的速度 / 力矩静默差数倍，位置倒是对的）；
3. 用**实测位置**使能 + 立刻补一帧保持帧，绝不朝零位冲；
4. 按 `--rate`（默认 100 Hz）连续发 0x300 帧：主机侧按 `--vel` 斜坡逼近目标，
   i_des = 力 × `--current-per-force`；
5. 每拍查 ERR / 反馈超时，任何异常立刻失能退出；
6. 退出（含 Ctrl-C）默认失能 —— **夹爪失能 = 松开，抓着的东西会掉**。

## 用法

    # 只看模式 / 档位 / 状态，不使能（第一次接上新电机先跑这个）
    pixi run python src/test/pvt_motor_test.py --check

    # 扫 0..15，看总线上到底有谁在（查 ID 配错 / 没上电）
    pixi run python src/test/pvt_motor_test.py --scan

    # 只失能（ROS 节点被强杀后电机还使能着顶住时，用这个放掉）
    pixi run python src/test/pvt_motor_test.py --disable

    # CTRL_MODE 不是 4 时，先切成 4（电机必须是失能状态；改模式会复位驱动器）
    pixi run python src/test/pvt_motor_test.py --set-mode 4

    # 交互：o 开 / c 闭 / h 保持 / +/- 调力 / e 使能 / d 失能 / q 退出
    pixi run python src/test/pvt_motor_test.py

    # 单次移动并保持 2 s（位置是**电机侧 rad**，夹爪全开 -5.572786、全闭 +0.393874）
    pixi run python src/test/pvt_motor_test.py --pos -5.572786 --vel 6 --force 12 --duration 2

    # 反复开合 10 次，带 CSV 记录（看 8N 到底够不够、齿条摩擦大不大）
    pixi run python src/test/pvt_motor_test.py --sweep 10 --vel 4 --force 10 --csv /tmp/grip.csv

## 单位与默认值

位置一律是**电机侧 rad**（输出轴），和 dm_frames.cpp 的 `force_pos_frame` 一致。
`--scale/--offset/--direction` 只影响显示（mm、夹紧力估计），不参与控制。
默认值就是末端夹爪 DM4310（ID 7，PMAX/VMAX/TMAX = 12.5/30/10，0.119 rad/mm，即 4 mm ≈ 0.48 rad）。
测别的电机：`--id 5 --pmax 12.5 --vmax 30 --tmax 10 --scale 0 --closed 1.5 --open -1.5`，
`--scale 0` 表示不做关节侧换算，只显示 rad。

## 注意

- 一个串口同一时刻只能一个进程：先关掉 ros2_control / 调试助手，否则 EBUSY 或互相踩数据；
- 本脚本**不写**除 CTRL_MODE 以外的任何寄存器，不动 ACC/DEC/ID，避免改了真机参数；
- 通信协议是 30 字节发送帧 / 16 字节接收帧，与 dm_frames.cpp 逐字节对齐。
"""

from __future__ import annotations

import argparse
import csv
import fcntl
import math
import os
import select
import struct
import sys
import termios
import time
import tty
import unicodedata
from dataclasses import dataclass

# ───────────────────────── 协议常量（与 dm_frames.hpp 一一对应）─────────────────────────
K_RX_LEN = 16                 # 适配器 → 主机（发送帧 30 字节，见 K_TX_TEMPLATE）

K_CMD_FEEDBACK = 0x11         # 反馈帧 CMD
K_CMD_ENABLE = 0xFC
K_CMD_DISABLE = 0xFD
K_CMD_REFRESH = 0xCC
K_CMD_REG_READ = 0x33
K_CMD_REG_WRITE = 0x55
K_CMD_REG_SAVE = 0xAA
K_REG_CMDS = (K_CMD_REG_READ, K_CMD_REG_WRITE, K_CMD_REG_SAVE)   # 绝不能当反馈解

K_CAN_ID_BROADCAST = 0x7FF    # 刷新帧 / 寄存器帧：目标 ID 在数据段里
K_CAN_ID_FORCE_POS = 0x300    # 力位混控 = 0x300 + slave_id（位置速度是 0x100 + id，本脚本不发）

K_REG_CTRL_MODE = 0x0A        # uint32：1=MIT 2=位置速度 3=速度 4=力位混控
K_REG_PMAX = 0x15             # float
K_REG_VMAX = 0x16             # float
K_REG_TMAX = 0x17             # float

K_CTRL_MODE_FORCE_POS = 4
K_VEL_MAX = 100.0             # v_des 的上限 rad/s（协议规定 0..100）
K_VEL_GAIN = 100.0            # v_des 放大 100 倍 → uint16
K_CURRENT_GAIN = 10000.0      # i_des 放大 10000 倍 → uint16

K_HOLD_VLIM = 0.1             # rad/s：使能后保持帧的速度给定（与 dm_joint.cpp 的 kHoldVlim 一致）

# 30 字节发送帧模板（[13:15] = CAN ID，[21:29] = 数据段）
# ⚠️ 不可推导的魔数：改错任何一个字节 = 适配器不认帧 = 电机不动，而且不会有任何报错。
K_TX_TEMPLATE = bytes((
    0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
))

ERR_TEXT = {
    0x0: "失能",
    0x1: "使能",
    0x8: "超压",
    0x9: "欠压",
    0xA: "过电流",
    0xB: "MOS 过温",
    0xC: "电机线圈过温",
    0xD: "通讯丢失（锁存，需断电）",
    0xE: "过载",
}


# ───────────────────────── 串口（只做字节进出）─────────────────────────
def _baud_table() -> dict[int, int]:
    table: dict[int, int] = {}
    for rate in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000):
        speed = getattr(termios, f"B{rate}", None)
        if speed is not None:
            table[rate] = speed
    return table


K_BAUD_TABLE = _baud_table()


class SerialPort:
    """非阻塞串口。与 dm_serial.cpp 一致：8N1、raw、TIOCEXCL 独占。"""

    def __init__(self, device: str, baud: int) -> None:
        self.device = device
        self.baud = baud
        self.fd = -1

    def open(self) -> None:
        speed = K_BAUD_TABLE.get(self.baud)
        if speed is None:
            raise RuntimeError(f"不支持的波特率 {self.baud}，可用：{sorted(K_BAUD_TABLE)}")
        try:
            fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise RuntimeError(f"打开串口失败 {self.device}：{exc.strerror}") from exc
        self.fd = fd

        # 独占：另一套 ros2_control 还开着这个口时直接报错，比两边互相踩数据好
        excl = getattr(termios, "TIOCEXCL", None)
        if excl is not None:
            try:
                fcntl.ioctl(fd, excl, 0)
            except OSError:
                pass

        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0                                              # iflag: raw
            attrs[1] = 0                                              # oflag: raw
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL    # 8N1、无流控
            attrs[3] = 0                                              # lflag: 无回显/非规范
            attrs[4] = speed                                          # ispeed
            attrs[5] = speed                                          # ospeed
            attrs[6][termios.VMIN] = 0                                # 非阻塞读
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except (OSError, termios.error) as exc:
            self.close()
            raise RuntimeError(f"配置串口失败 {self.device}：{exc}") from exc

        termios.tcflush(fd, termios.TCIFLUSH)   # 丢掉打开前就堆着的陈旧字节

    def close(self) -> None:
        if self.fd >= 0:
            try:
                os.close(self.fd)
            finally:
                self.fd = -1

    def write(self, data: bytes) -> None:
        if self.fd < 0:
            raise RuntimeError("串口没打开，不能写")
        view = memoryview(data)
        while view:
            try:
                written = os.write(self.fd, view)
            except BlockingIOError:
                if not select.select([], [self.fd], [], 0.1)[1]:
                    raise RuntimeError("写串口超时")
                continue
            except OSError as exc:
                raise RuntimeError(f"写串口失败：{exc.strerror}") from exc
            view = view[written:]

    def read_available(self) -> bytes:
        if self.fd < 0:
            raise RuntimeError("串口没打开，不能读")
        chunks: list[bytes] = []
        while True:
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                break
            except OSError as exc:
                raise RuntimeError(f"读串口失败：{exc.strerror}") from exc
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)


class RxBuffer:
    """切 16 字节帧；不完整的尾巴留到下一次，绝不丢。"""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> None:
        self._buf.extend(chunk)

    def drain(self) -> list[bytes]:
        buf = self._buf
        frames: list[bytes] = []
        i = 0
        while i + K_RX_LEN <= len(buf):
            if buf[i] == 0xAA and buf[i + K_RX_LEN - 1] == 0x55:
                frames.append(bytes(buf[i:i + K_RX_LEN]))
                i += K_RX_LEN
            else:
                i += 1
        if i:
            del buf[:i]
        return frames


# ───────────────────────── 帧构造（逐字节对齐 dm_frames.cpp）─────────────────────────
def _build_tx(can_id: int, data: bytes) -> bytes:
    if len(data) != 8:
        raise ValueError(f"数据段必须 8 字节，收到 {len(data)}")
    frame = bytearray(K_TX_TEMPLATE)
    frame[13] = can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[21:29] = data
    return bytes(frame)


def _unsigned_raw(value: float, hi: float, gain: float) -> int:
    """无符号定点：先钳到 [0, hi] 再乘增益——与 C++ 侧 max(0, min(x, hi)) 一致。"""
    clamped = 0.0 if value <= 0.0 else (hi if value > hi else value)
    return int(clamped * gain)


def force_pos_frame(motor_id: int, p_des: float, v_des: float, i_des: float) -> bytes:
    """0x300+ID：float32 位置 + uint16 限速 + uint16 电流上限。"""
    if not (math.isfinite(p_des) and math.isfinite(v_des) and math.isfinite(i_des)):
        raise ValueError("力位混控帧里有 NaN/inf，拒发")
    data = struct.pack(
        "<fHH",
        p_des,
        _unsigned_raw(v_des, K_VEL_MAX, K_VEL_GAIN),
        _unsigned_raw(i_des, 1.0, K_CURRENT_GAIN),
    )
    return _build_tx(K_CAN_ID_FORCE_POS + motor_id, data)


def cmd_frame(motor_id: int, cmd: int) -> bytes:
    """使能 0xFC / 失能 0xFD：数据段 FF×7 + 命令字。"""
    if cmd not in (K_CMD_ENABLE, K_CMD_DISABLE):
        raise ValueError(f"只支持使能 0xFC / 失能 0xFD，收到 0x{cmd:02X}")
    return _build_tx(motor_id, bytes((0xFF,) * 7 + (cmd,)))


def refresh_frame(motor_id: int) -> bytes:
    """0x7FF + [ID_L, ID_H, 0xCC, ...]：只问状态，不改控制。"""
    return _build_tx(
        K_CAN_ID_BROADCAST,
        bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, K_CMD_REFRESH, 0, 0, 0, 0, 0)),
    )


def reg_read_frame(motor_id: int, rid: int) -> bytes:
    return _build_tx(
        K_CAN_ID_BROADCAST,
        bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, K_CMD_REG_READ, rid, 0, 0, 0, 0)),
    )


def reg_write_frame(motor_id: int, rid: int, payload: bytes) -> bytes:
    if len(payload) != 4:
        raise ValueError("寄存器写必须 4 字节")
    return _build_tx(
        K_CAN_ID_BROADCAST,
        bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, K_CMD_REG_WRITE, rid)) + payload,
    )


# ───────────────────────── 帧解码 ─────────────────────────
@dataclass(frozen=True)
class Limit:
    p_max: float
    v_max: float
    t_max: float


@dataclass
class Feedback:
    motor_id: int
    err: int
    pos: float        # 电机侧 rad
    vel: float        # 电机侧 rad/s
    tau: float        # 电机侧 N·m（驱动估计值）
    t_mos: int        # ℃
    t_rotor: int      # ℃
    stamp: float      # monotonic


def is_reg_response(frame: bytes) -> bool:
    """寄存器回包与反馈帧共用 CMD=0x11，必须先分流（与 dm_frames.cpp 一致）。"""
    return (
        frame[1] == K_CMD_FEEDBACK
        and frame[9] in K_REG_CMDS
        and frame[7] <= 0x0F
        and frame[8] == 0x00
    )


def _u_to_float(raw: int, bits: int, low: float, high: float) -> float:
    return raw / ((1 << bits) - 1) * (high - low) + low


def decode_feedback(frame: bytes, limit: Limit) -> Feedback:
    data = frame[7:15]
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    tau_raw = ((data[4] & 0x0F) << 8) | data[5]
    return Feedback(
        motor_id=data[0] & 0x0F,
        err=(data[0] >> 4) & 0x0F,
        pos=_u_to_float(pos_raw, 16, -limit.p_max, limit.p_max),
        vel=_u_to_float(vel_raw, 12, -limit.v_max, limit.v_max),
        tau=_u_to_float(tau_raw, 12, -limit.t_max, limit.t_max),
        t_mos=data[6],
        t_rotor=data[7],
        stamp=time.monotonic(),
    )


def decode_reg_response(frame: bytes) -> tuple[int, int, int, bytes]:
    """返回 (motor_id, cmd, rid, payload4)。"""
    data = frame[7:15]
    return (data[1] << 8) | data[0], data[2], data[3], bytes(data[4:8])


# ───────────────────────── 单台电机 ─────────────────────────
class Motor:
    """一条总线上的**一台**电机：收发、反馈缓存、寄存器读写、使能/失能、发 0x300 帧。

    边界：不拥有控制循环（send 只发一帧）、不判安全（ERR / 超时由 Tester 判）。
    """

    def __init__(self, port: SerialPort, motor_id: int, limit: Limit) -> None:
        self.port = port
        self.id = motor_id
        self.limit = limit
        self.rx = RxBuffer()
        self.state: Feedback | None = None
        self.seen: set[int] = set()           # 总线上出现过的所有 ID
        self.reg_responses: list[tuple[int, int, bytes]] = []

    # ── 收发 ──
    def pump(self) -> None:
        chunk = self.port.read_available()
        if not chunk:
            return
        self.rx.feed(chunk)
        for frame in self.rx.drain():
            self._ingest(frame)

    def _ingest(self, frame: bytes) -> None:
        if frame[1] != K_CMD_FEEDBACK:
            return
        if is_reg_response(frame):
            motor_id, _cmd, rid, payload = decode_reg_response(frame)
            self.reg_responses.append((motor_id, rid, payload))
            return
        fb = decode_feedback(frame, self.limit)
        self.seen.add(fb.motor_id)
        if fb.motor_id == self.id:
            self.state = fb

    def fresh(self, timeout: float = 0.5) -> bool:
        return self.state is not None and (time.monotonic() - self.state.stamp) < timeout

    # ── 控制帧 ──
    def refresh(self) -> None:
        self.port.write(refresh_frame(self.id))

    def send(self, p_des: float, v_des: float, i_des: float) -> None:
        self.port.write(force_pos_frame(self.id, p_des, v_des, i_des))

    def hold(self, i_des: float) -> None:
        """保持帧：用**实测位置**，速度给很小的值，不经过软限位/换算。"""
        if self.state is None:
            raise RuntimeError("没有实测位置，不能发保持帧")
        self.send(self.state.pos, K_HOLD_VLIM, i_des)

    def enable(self, i_des: float) -> None:
        """先确认有实测位置 → 使能 → 立刻补保持帧（防止朝零位冲）。"""
        if not self.fresh(0.5):
            raise RuntimeError(
                "没有新鲜的位置反馈，拒绝使能：位置类模式下驱动器内部指令是 0，"
                "此时使能电机会朝零位冲。先让总线收到反馈。"
            )
        self.port.write(cmd_frame(self.id, K_CMD_ENABLE))
        self.hold(i_des)

    def disable(self) -> None:
        self.port.write(cmd_frame(self.id, K_CMD_DISABLE))

    # ── 寄存器（只在使能前用；读的时候会停发控制帧）──
    def read_register(self, rid: int, timeout: float = 0.15, retries: int = 3) -> bytes | None:
        for _ in range(retries):
            self.reg_responses.clear()
            self.port.write(reg_read_frame(self.id, rid))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.pump()
                for motor_id, got_rid, payload in self.reg_responses:
                    if motor_id == self.id and got_rid == rid:
                        self.reg_responses.clear()
                        return payload
                time.sleep(0.002)
        return None

    def read_u32(self, rid: int) -> int | None:
        payload = self.read_register(rid)
        return None if payload is None else struct.unpack("<I", payload)[0]

    def read_float(self, rid: int) -> float | None:
        payload = self.read_register(rid)
        return None if payload is None else struct.unpack("<f", payload)[0]

    def write_u32(self, rid: int, value: int) -> int | None:
        """写一个 uint32 寄存器，返回回读值（写参数会让驱动器复位，所以等一会儿再读）。"""
        self.port.write(reg_write_frame(self.id, rid, struct.pack("<I", value)))
        time.sleep(0.1)
        return self.read_u32(rid)

    # ── 扫描 ──
    def scan(self, duration: float = 0.6) -> set[int]:
        self.seen.clear()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            for motor_id in range(16):
                self.port.write(refresh_frame(motor_id))
            time.sleep(0.02)
            self.pump()
        if self.state is not None:
            self.seen.add(self.id)
        return set(self.seen)


# ───────────────────────── 显示小工具 ─────────────────────────
def _display_width(text: str) -> int:
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)


def pad(text: object, width: int, align: str = "<") -> str:
    """按终端显示宽度补空格（中文占 2 格），让表头和数据对齐。"""
    string = str(text)
    space = width - _display_width(string)
    if space <= 0:
        return string
    return string + " " * space if align == "<" else " " * space + string


def clamp(value: float, low: float, high: float) -> float:
    if value <= low:
        return low
    if value > high:
        return high
    return value


def fmt(value: float | None, digits: int = 4) -> str:
    if value is None:
        return "—"
    if abs(value) < 5e-7:
        value = 0.0
    return f"{value:+.{digits}f}"


# ───────────────────────── 测试器 ─────────────────────────
class Tester:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.port = SerialPort(args.port, args.baud)
        self.motor = Motor(self.port, args.id, Limit(args.pmax, args.vmax, args.tmax))

        # 软限位：在 open/closed 之间再往里让 margin；目标只许落在软限位内。
        # 但使能瞬间要能"原地保持"——所以命令值本身允许在完整行程内（见 tick）。
        self.lo_travel = min(args.closed, args.open)
        self.hi_travel = max(args.closed, args.open)
        self.lo = self.lo_travel + args.margin
        self.hi = self.hi_travel - args.margin
        if self.lo >= self.hi:
            raise SystemExit("--closed/--open 与 --margin 不成立：可用行程为空")

        self.force = clamp(args.force, 0.0, args.force_max)
        self.mode: int | None = None
        self.limit_warnings: list[str] = []

        self.enabled = False
        self.cmd: float | None = None
        self.target: float | None = None
        self.ticks = 0
        self.max_follow_err = 0.0
        self.move_max_err = 0.0
        self.last_refresh = 0.0
        self.next_tx = 0.0
        self.last_tick = time.monotonic()
        self.t0 = time.monotonic()

        self.quit = False
        self.message = ""
        self.message_until = 0.0
        self._saved_termios = None

        self.csv_file = None
        self.csv_writer = None
        if args.csv:
            self.csv_file = open(args.csv, "w", newline="", buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "t_s", "target_rad", "cmd_rad", "pos_rad", "vel_radps", "tau_nm",
                "force_n_est", "t_mos_c", "t_rotor_c", "err", "i_des",
            ])

    # ── 换算（只影响显示）──
    def current(self) -> float:
        if self.args.i_des is not None:
            return clamp(self.args.i_des, 0.0, 1.0)
        return clamp(self.force * self.args.current_per_force, 0.0, 1.0)

    def finger_mm(self, motor_rad: float) -> float | None:
        if self.args.scale <= 0.0:
            return None
        return self.args.direction * (motor_rad - self.args.offset) / self.args.scale * 1000.0

    def force_n_est(self, tau: float) -> float | None:
        if self.args.scale <= 0.0:
            return None
        return abs(tau) * self.args.scale

    # ── 连接与自检 ──
    def wait_feedback(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        last_refresh = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_refresh >= 0.05:
                self.motor.refresh()
                last_refresh = now
            self.motor.pump()
            if self.motor.fresh(0.3):
                return
            time.sleep(0.002)
        raise RuntimeError(
            f"{timeout:.1f}s 内没收到 ID {self.motor.id} 的反馈：\n"
            "  1) 串口选对了吗（ls /dev/ttyACM*）；\n"
            "  2) 电机供电了吗、CAN 波特率是不是 1 Mbps；\n"
            "  3) ID 是不是这个（先跑 --scan 看看总线上有谁）；\n"
            "  4) 串口被别的进程占了（ros2_control / 调试助手）？"
        )

    def check_mode(self) -> None:
        if self.args.skip_mode_check:
            print("警告：--skip-mode-check —— 模式不对时 0x300 帧会被静默忽略，自己确认 0x0A=4")
            return

        mode = self.motor.read_u32(K_REG_CTRL_MODE)
        if mode == K_CTRL_MODE_FORCE_POS:
            self.mode = mode
            return
        if self.args.set_mode is None:
            what = "读不到 CTRL_MODE" if mode is None else f"CTRL_MODE={mode}"
            raise RuntimeError(
                f"{what}，不是 4（力位混控）。模式不对时位置帧会被**静默忽略**："
                "使能正常、反馈正常、电机不动。\n"
                "  加 --set-mode 4 切换（要求电机失能），再重新跑本脚本。"
            )
        if self.motor.state is not None and self.motor.state.err == 0x1:
            raise RuntimeError(
                "电机当前使能中。改控制模式会让驱动器复位，"
                "先失能（或断电）再执行 --set-mode。"
            )

        got = self.motor.write_u32(K_REG_CTRL_MODE, self.args.set_mode)
        if got != self.args.set_mode:
            raise RuntimeError(f"写 CTRL_MODE={self.args.set_mode} 后回读 {got}，没生效（寄存器 0x0A）")
        print(
            f"已把 CTRL_MODE 写成 {got}"
            + ("（力位混控）" if got == K_CTRL_MODE_FORCE_POS else "")
            + "；写参数会复位驱动器，重新等反馈…"
        )
        time.sleep(0.2)
        self.wait_feedback(self.args.connect_timeout)
        self.mode = got

    def check_limits(self) -> list[str]:
        warnings: list[str] = []
        for name, rid, expect in (
            ("PMAX", K_REG_PMAX, self.args.pmax),
            ("VMAX", K_REG_VMAX, self.args.vmax),
            ("TMAX", K_REG_TMAX, self.args.tmax),
        ):
            value = self.motor.read_float(rid)
            if value is None:
                warnings.append(f"{name}(0x{rid:02X}) 读不到，反馈缩放没法核对")
            elif abs(value - expect) > max(0.01, abs(expect) * 0.01):
                warnings.append(
                    f"{name} 回读 {value:g} ≠ 命令行 {expect:g}：反馈的速度/力矩会按错档位解码，"
                    "把命令行改成回读值"
                )
        return warnings

    def connect(self) -> None:
        print(f"打开 {self.args.port} @ {self.args.baud}，等 ID {self.args.id} 的反馈…")
        self.wait_feedback(self.args.connect_timeout)
        self.check_mode()
        self.limit_warnings = self.check_limits()
        self.print_status()

    def print_status(self) -> None:
        st = self.motor.state
        assert st is not None
        mode_text = "未检查" if self.mode is None else str(self.mode)
        mm = self.finger_mm(st.pos)
        print(
            f"电机 ID {st.motor_id}  CTRL_MODE={mode_text}  "
            f"PMAX/VMAX/TMAX={self.args.pmax:g}/{self.args.vmax:g}/{self.args.tmax:g}  状态 {ERR_TEXT.get(st.err, '?')}"
        )
        print(
            f"位置 {fmt(st.pos)} rad"
            + (f"（{fmt(mm, 2)} mm）" if mm is not None else "")
            + f"   速度 {fmt(st.vel, 2)} rad/s   力矩 {fmt(st.tau, 3)} N·m"
            f"   温度 MOS {st.t_mos}℃ / 线圈 {st.t_rotor}℃"
        )
        for warning in self.limit_warnings:
            print(f"[!] {warning}")

    # ── 控制 ──
    def set_message(self, text: str, seconds: float = 8.0) -> None:
        self.message = text
        self.message_until = time.monotonic() + seconds

    def enable(self) -> None:
        sent_at = time.monotonic()
        self.motor.enable(self.current())     # 已发使能帧 + 保持帧
        self.enabled = True                   # 从这里开始，收尾必须失能（即使确认失败）
        # 等一帧"使能之后"的反馈再进控制循环：缓存里的上一帧还是 err=0x0，
        # 直接用它判故障会误报"电机已失能"（真机上反馈也比命令晚一拍）。
        deadline = sent_at + 0.5
        while time.monotonic() < deadline:
            self.motor.pump()
            st = self.motor.state
            if st is not None and st.err == 0x1 and st.stamp >= sent_at:
                break
            time.sleep(0.002)
        else:
            raise RuntimeError(
                "使能后 0.5s 没看到 ERR=0x1：使能没生效，或者使能后反馈就断了；"
                "退出时会补发失能帧"
            )
        # 原地保持：命令 = 目标 = 实测位置，使能瞬间不产生任何位移
        self.last_tick = time.monotonic()
        self.cmd = self.motor.state.pos
        self.target = self.cmd
        self.move_max_err = 0.0
        self.set_message("已使能，保持当前位置")

    def disable(self) -> None:
        if not self.enabled:
            return
        self.motor.disable()
        self.enabled = False
        self.set_message("已失能（夹爪会松开）")

    def tick(self, now: float) -> None:
        st = self.motor.state
        if not self.enabled:
            if now - self.last_refresh >= 0.05:
                self.motor.refresh()
                self.last_refresh = now
            return
        if st is None or now - st.stamp > self.args.feedback_timeout:
            raise RuntimeError(
                f"反馈超时（>{self.args.feedback_timeout:g}s）：看不到电机状态就不敢继续发位置命令"
            )
        if st.err == 0x0:
            raise RuntimeError("电机已经回到失能状态（ERR=0x0）：可能通讯丢失/驱动器复位，退出")
        if st.err != 0x1:
            raise RuntimeError(f"电机报错 ERR=0x{st.err:X}（{ERR_TEXT.get(st.err, '未知')}）")

        assert self.cmd is not None and self.target is not None
        # dt 必须取"距上一条控制帧"的时间：用循环迭代时间（~1 ms）会让斜坡慢一个数量级
        dt = min(now - self.last_tick, 0.1)
        self.last_tick = now
        step = self.args.vel * dt
        delta = self.target - self.cmd
        self.cmd = self.target if abs(delta) <= step else self.cmd + math.copysign(step, delta)
        # 命令值允许落在完整行程内（使能瞬间可能正好在硬限位上），目标值另有软限位
        self.cmd = clamp(self.cmd, self.lo_travel, self.hi_travel)

        self.motor.send(self.cmd, self.args.vel, self.current())
        self.ticks += 1
        self.max_follow_err = max(self.max_follow_err, abs(self.cmd - st.pos))
        self.move_max_err = max(self.move_max_err, abs(self.cmd - st.pos))
        self.log_row(now, st)

    def log_row(self, now: float, st: Feedback) -> None:
        if self.csv_writer is None:
            return
        force_est = self.force_n_est(st.tau)
        self.csv_writer.writerow([
            f"{now - self.t0:.4f}",
            f"{self.target:.5f}" if self.target is not None else "",
            f"{self.cmd:.5f}" if self.cmd is not None else "",
            f"{st.pos:.5f}", f"{st.vel:.4f}", f"{st.tau:.4f}",
            "" if force_est is None else f"{force_est:.3f}",
            st.t_mos, st.t_rotor, st.err, f"{self.current():.5f}",
        ])

    def step_control(self, now: float) -> None:
        if now >= self.next_tx:
            self.tick(now)
            self.next_tx += 1.0 / self.args.rate
            if self.next_tx < now:          # 落后就不追帧，直接对齐到现在
                self.next_tx = now + 1.0 / self.args.rate
        self.motor.pump()

    def arrived(self) -> bool:
        st = self.motor.state
        if st is None or self.target is None or self.cmd is None:
            return False
        return abs(self.target - self.cmd) < 1e-9 and abs(st.pos - self.target) <= self.args.tolerance

    def set_target(self, target: float, label: str) -> None:
        """只改目标，不阻塞——交互模式下必须这样，否则移动期间按不了空格急停。"""
        self.target = clamp(target, self.lo, self.hi)
        self.move_max_err = 0.0
        self.set_message(f"{label} → {fmt(self.target)} rad")

    def goto(self, target: float, label: str) -> bool:
        self.set_target(target, label)
        assert self.target is not None
        timeout = max(2.0, abs(self.target - (self.cmd or 0.0)) / max(self.args.vel, 1e-3) * 2.0 + 1.0)
        return self.spin_until(self.arrived, timeout)

    def spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            self.step_control(now)
            if predicate():
                return True
            time.sleep(0.001)
        return False

    def hold(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.step_control(time.monotonic())
            time.sleep(0.001)

    # ── 运行方式 ──
    def run_disable(self) -> None:
        """只发失能帧：节点被强杀 / 停止脚本后电机还使能着顶住时，用这个放掉。"""
        self.motor.disable()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.motor.refresh()
            self.motor.pump()
            st = self.motor.state
            if st is not None and st.err == 0x0:
                print(f"已失能（ERR=0x0），当前位置 {fmt(st.pos)} rad")
                return
            time.sleep(0.02)
        st = self.motor.state
        if st is None:
            print("已发失能帧，但没收到反馈 —— 确认串口/电源")
        else:
            print(f"已发失能帧，但电机仍报 ERR=0x{st.err:X}（{ERR_TEXT.get(st.err, '未知')}），检查是否被别的主机占着")

    def run_scan(self) -> None:
        seen = self.motor.scan()
        print(f"扫到 {len(seen)} 台电机：" if seen else "总线上一台都没扫到")
        for motor_id in sorted(seen):
            tag = "  ← 本次目标" if motor_id == self.motor.id else ""
            print(f"  ID {motor_id}{tag}")
        if not seen:
            print(
                "排查：适配器供电 / 电机供电 / CAN 线 / 波特率（默认 1 Mbps）/ 串口是否选对"
            )

    def run_check(self) -> None:
        st = self.motor.state
        assert st is not None
        print("\n自检通过（未使能，电机保持原状态）。")
        if st.err == 0x1:
            print("[!] 电机当前是使能状态：手摆会被电机对抗，先断电/失能。")
        print(f"可用行程（已让 {self.args.margin:g} rad 边距）：{fmt(self.lo)} .. {fmt(self.hi)} rad")

    def run_single(self) -> None:
        target = clamp(self.args.pos, self.lo, self.hi)
        if abs(target - self.args.pos) > 1e-9:
            print(f"目标 {fmt(self.args.pos)} rad 超出软限位，钳到 {fmt(target)} rad")
        before = self.motor.state.pos
        print(f"移动 {fmt(before)} → {fmt(target)} rad，速度 {self.args.vel:g} rad/s，"
              f"力 {self.force:g} N（i_des {self.current():.5f}）")
        ok = self.goto(target, "移动")
        self.hold(self.args.duration)
        st = self.motor.state
        print(
            f"{'已到位' if ok else '超时没到位'}：目标 {fmt(target)}，实测 {fmt(st.pos)}，"
            f"误差 {fmt(st.pos - target)} rad，最大跟随误差 {fmt(self.move_max_err)} rad"
        )
        print(f"速度 {fmt(st.vel, 2)} rad/s，力矩 {fmt(st.tau, 3)} N·m，"
              f"温度 MOS {st.t_mos}℃ / 线圈 {st.t_rotor}℃，ERR 0x{st.err:X}")

    def run_sweep(self) -> None:
        cycles = self.args.sweep
        print(f"开合 {cycles} 次：{fmt(self.args.closed)} ⇄ {fmt(self.args.open)} rad，"
              f"速度 {self.args.vel:g} rad/s，力 {self.force:g} N")
        cycle_times: list[float] = []
        peak_err = 0.0
        for index in range(cycles):
            started = time.monotonic()
            ok_open = self.goto(self.args.open, "开")
            err_open = self.move_max_err
            ok_close = self.goto(self.args.closed, "闭")
            err_close = self.move_max_err
            elapsed = time.monotonic() - started
            cycle_times.append(elapsed)
            peak_err = max(peak_err, err_open, err_close)
            print(
                f"  #{index + 1:02d}  {elapsed:5.2f}s  到位 {'✓' if ok_open and ok_close else '✗'}"
                f"  最大跟随误差 {max(err_open, err_close):.4f} rad"
            )
        st = self.motor.state
        if cycle_times:
            print(
                f"完成 {len(cycle_times)} 次，平均 {sum(cycle_times) / len(cycle_times):.2f}s/次，"
                f"峰值跟随误差 {peak_err:.4f} rad"
            )
        print(f"温度 MOS {st.t_mos}℃ / 线圈 {st.t_rotor}℃，ERR 0x{st.err:X}")
        if self.csv_file is not None:
            print(f"CSV 已写入 {self.args.csv}")

    # ── 交互 ──
    def keys_help(self) -> list[str]:
        return [
            "[o] 开到 --open    [c] 闭到 --closed    [h] 保持当前位置",
            f"[+/-] 夹紧力 ±2N（当前 {self.force:g} N，上限 {self.args.force_max:g} N，i_des {self.current():.5f}）",
            "[e] 使能    [d] 失能    [q] 退出（自动失能）",
        ]

    def draw(self) -> None:
        st = self.motor.state
        assert st is not None
        mm = self.finger_mm(st.pos)
        cmd_mm = self.finger_mm(self.cmd) if self.cmd is not None else None
        force_est = self.force_n_est(st.tau)
        state_text = "使能" if st.err == 0x1 else ERR_TEXT.get(st.err, f"0x{st.err:X}")
        if not self.motor.fresh(self.args.feedback_timeout):
            state_text += "（反馈中断）"
        if self.arrived():
            state_text += " · 已到位"

        lines = [
            f"PVT 力位混控测试 — 电机 ID {st.motor_id}  CTRL_MODE={self.mode if self.mode is not None else '未检查'}"
            f"  串口 {self.port.device} @ {self.port.baud}  {self.args.rate:g} Hz",
            f"位置  指令 {fmt(self.cmd)} rad"
            + (f"（{fmt(cmd_mm, 2)} mm）" if cmd_mm is not None else "")
            + f"   实测 {fmt(st.pos)} rad"
            + (f"（{fmt(mm, 2)} mm）" if mm is not None else "")
            + f"   误差 {fmt(None if self.cmd is None else self.cmd - st.pos)} rad",
            f"速度  给定 {fmt(self.args.vel, 2)} rad/s   实测 {fmt(st.vel, 2)} rad/s",
            f"力矩  {fmt(st.tau, 3)} N·m"
            + (f"   夹紧力估计 {force_est:.1f} N" if force_est is not None else "")
            + f"   温度 MOS {st.t_mos}℃ / 线圈 {st.t_rotor}℃",
            f"状态  {state_text}    已发 {self.ticks} 帧    最大跟随误差 {self.max_follow_err:.4f} rad",
            "",
        ]
        lines.extend(self.keys_help())
        for warning in self.limit_warnings:
            lines.append(f"[!] {warning}")
        if self.message and time.monotonic() < self.message_until:
            lines.append("» " + self.message)
        sys.stdout.write("\x1b[H" + "\n".join(line + "\x1b[K" for line in lines) + "\x1b[J")
        sys.stdout.flush()

    def enter_terminal(self) -> None:
        self._saved_termios = termios.tcgetattr(0)
        tty.setcbreak(0)
        sys.stdout.write("\x1b[?25l\x1b[2J")
        sys.stdout.flush()

    def leave_terminal(self) -> None:
        if self._saved_termios is not None:
            termios.tcsetattr(0, termios.TCSADRAIN, self._saved_termios)
            self._saved_termios = None
            sys.stdout.write("\x1b[?25h\n")
            sys.stdout.flush()

    def handle_key(self, key: str) -> None:
        if key in ("q", "Q"):
            self.quit = True
        elif key in ("o", "O"):
            self.set_target(self.args.open, "开")
        elif key in ("c", "C"):
            self.set_target(self.args.closed, "闭")
        elif key in ("h", "H"):
            if self.motor.state is not None:
                self.set_target(self.motor.state.pos, "保持")
        elif key in ("+", "="):
            if self.args.i_des is not None:
                self.set_message("--i-des 已经把电流固定了，力调节无效")
                return
            self.force = clamp(self.force + 2.0, 0.0, self.args.force_max)
            self.set_message(f"夹紧力 {self.force:g} N（i_des {self.current():.5f}）")
        elif key in ("-", "_"):
            if self.args.i_des is not None:
                self.set_message("--i-des 已经把电流固定了，力调节无效")
                return
            self.force = clamp(self.force - 2.0, 0.0, self.args.force_max)
            self.set_message(f"夹紧力 {self.force:g} N（i_des {self.current():.5f}）")
        elif key in ("e", "E"):
            if not self.enabled:
                try:
                    self.enable()
                except RuntimeError as exc:
                    self.set_message(f"使能失败：{exc}")
        elif key in ("d", "D"):
            self.disable()
        elif key == " ":
            self.disable()
            self.set_message("空格：立即失能")

    def run_interactive(self) -> None:
        if not sys.stdin.isatty():
            raise SystemExit("交互模式需要终端；用 --check / --pos / --sweep 走非交互模式")
        self.enter_terminal()
        try:
            self.next_tx = time.monotonic()
            self.last_tick = self.next_tx
            last_draw = -1.0
            while not self.quit:
                now = time.monotonic()
                self.step_control(now)

                readable, _, _ = select.select([0], [], [], 0)
                if readable:
                    try:
                        keys = os.read(0, 32)
                    except OSError:
                        keys = b""
                    for key in keys.decode(errors="ignore"):
                        self.handle_key(key)

                if now - last_draw >= 0.1:
                    self.draw()
                    last_draw = now
                time.sleep(0.001)
        finally:
            self.leave_terminal()

    # ── 收尾 ──
    def shutdown(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        if not self.enabled:
            return
        if self.args.keep_enabled:
            print("保持使能（--keep-enabled）：电机继续顶在当前位置，记得自己失能")
            return
        try:
            self.motor.disable()
            print("已失能（夹爪松开；抓着的东西会掉）")
        except RuntimeError as exc:
            print(f"失能失败：{exc}（请直接断电）", file=sys.stderr)

    def run(self) -> int:
        try:
            self.port.open()
        except RuntimeError as exc:
            print(f"错误：{exc}", file=sys.stderr)
            self.shutdown()        # 关掉可能已经打开的 CSV
            return 1

        code = 0
        try:
            if self.args.scan:
                self.run_scan()
                return 0
            if self.args.disable:
                self.run_disable()
                return 0
            self.connect()
            if self.args.check:
                self.run_check()
                return 0
            self.enable()
            if self.args.pos is not None:
                self.run_single()
            elif self.args.sweep:
                self.run_sweep()
            else:
                self.run_interactive()
        except KeyboardInterrupt:
            print("\nCtrl-C：退出")
        except RuntimeError as exc:
            print(f"错误：{exc}", file=sys.stderr)
            code = 1
        finally:
            self.shutdown()
            self.port.close()
        return code


# ───────────────────────── 命令行 ─────────────────────────
def build_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="达妙电机 PVT（力位混控 CTRL_MODE=4，0x300+ID）单机测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "位置一律是电机侧 rad。夹爪默认值：全闭 +0.393874、全开 -5.572786。\n"
            "第一次接新电机：先 --check 看模式和档位，再交互 o/c 试。"
        ),
    )
    # 通信
    parser.add_argument("--port", default="/dev/ttyACM0", help="串口设备（默认 /dev/ttyACM0）")
    parser.add_argument("--baud", type=int, default=921600, help="串口波特率（默认 921600）")
    parser.add_argument("--id", type=int, default=7, help="电机 CAN ID（默认 7，末端夹爪）")

    # 档位（必须与寄存器回读一致）
    parser.add_argument("--pmax", type=float, default=12.5, help="PMAX（默认 12.5）")
    parser.add_argument("--vmax", type=float, default=30.0, help="VMAX（默认 30，4310）")
    parser.add_argument("--tmax", type=float, default=10.0, help="TMAX（默认 10，4310）")

    # 关节侧换算（只影响显示）
    parser.add_argument("--scale", type=float, default=119.3332,
                        help="电机 rad / 关节单位（夹爪 119.3332；0 = 不做换算，只显示 rad）")
    parser.add_argument("--offset", type=float, default=0.393874, help="零位偏移 rad（夹爪 0.393874）")
    parser.add_argument("--direction", type=int, default=-1, choices=(-1, 1), help="方向（夹爪 -1）")

    # 行程与力
    parser.add_argument("--closed", type=float, default=0.393874, help="闭合位置 rad（默认 +0.393874）")
    parser.add_argument("--open", type=float, default=-5.572786, help="全开位置 rad（默认 -5.572786）")
    parser.add_argument("--margin", type=float, default=0.02, help="软限位内缩 rad（默认 0.02）")

    # 运动参数
    parser.add_argument("--vel", type=float, default=6.0, help="限速 rad/s（v_des，默认 6）")
    parser.add_argument("--force", type=float, default=8.0, help="夹紧力 N（默认 8）")
    parser.add_argument("--force-max", type=float, default=20.0, help="夹紧力上限 N（默认 20）")
    parser.add_argument("--current-per-force", type=float, default=0.00062,
                        help="i_des / N（默认 0.00062，近似值，可用秤标定）")
    parser.add_argument("--i-des", type=float, default=None,
                        help="直接给电流标幺 0..1，覆盖 --force 换算（查驱动时用）")
    parser.add_argument("--rate", type=float, default=100.0, help="控制帧频率 Hz（默认 100）")
    parser.add_argument("--tolerance", type=float, default=0.1, help="到位判据 rad（默认 0.1）")
    parser.add_argument("--feedback-timeout", type=float, default=0.35,
                        help="反馈超时 s，超过就失能退出（默认 0.35）")
    parser.add_argument("--connect-timeout", type=float, default=1.0, help="等首帧反馈 s（默认 1.0）")

    # 动作
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--pos", type=float, default=None, help="单次移动到该位置 rad 后退出")
    group.add_argument("--sweep", type=int, default=0, help="开合循环次数")
    parser.add_argument("--duration", type=float, default=1.0, help="到目标后保持 s（默认 1.0）")
    parser.add_argument("--csv", default=None, help="记录 CSV 路径")
    parser.add_argument("--keep-enabled", action="store_true", help="退出时不失能（默认失能）")

    # 自检
    parser.add_argument("--check", action="store_true", help="只看模式/档位/状态，不使能")
    parser.add_argument("--scan", action="store_true", help="扫 0..15 看总线上有谁")
    parser.add_argument("--disable", action="store_true",
                        help="只发失能帧后退出（电机卡在使能状态时的救援，不使能、不动）")
    parser.add_argument("--set-mode", type=int, default=None,
                        help="把 CTRL_MODE 写成该值（通常是 4；要求电机失能）")
    parser.add_argument("--skip-mode-check", action="store_true",
                        help="跳过 CTRL_MODE 检查（老固件读不到 0x0A 时用，风险自负）")

    args = parser.parse_args()
    if args.id < 0 or args.id > 0x0F:
        parser.error("电机 ID 必须在 0..15 之间（反馈帧只回 4 位 ID）")
    if args.rate <= 0:
        parser.error("--rate 必须 > 0")
    if args.vel <= 0 or args.vel > K_VEL_MAX:
        parser.error(f"--vel 必须在 (0, {K_VEL_MAX:g}] rad/s 之间")
    return args


def main() -> int:
    return Tester(build_args()).run()


if __name__ == "__main__":
    sys.exit(main())
