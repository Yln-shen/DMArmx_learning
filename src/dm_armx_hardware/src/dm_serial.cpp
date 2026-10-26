// dm_serial.cpp —— termios 串口实现（非阻塞读、写满、不 sleep）。
#include "dm_armx_hardware/dm_serial.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace dm_armx_hardware
{

namespace
{

// 只支持工程用到的这几档；适配器是 CDC-ACM，波特率本身对固件无意义，但**必须设成功**才开得了口
speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("不支持的波特率：" + std::to_string(baud));
  }
}

std::string errno_text(const char * what)
{
  return std::string(what) + "：" + std::strerror(errno);
}

}  // namespace

SerialPort::SerialPort(std::string device, int baud)
: device_(std::move(device)), baud_(baud)
{
  (void)baud_to_speed(baud_);   // 早点拒掉不支持的档位（构造即失败，别等 open）
}

SerialPort::~SerialPort()
{
  try {
    close();
  } catch (...) {
    // 析构不抛
  }
}

void SerialPort::open()
{
  if (fd_ >= 0) {return;}
  const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {throw std::runtime_error(errno_text(("打开串口失败 " + device_).c_str()));}
  fd_ = fd;

  // 独占这个口：**同一时刻只允许一套节点开它**。
  // 两个 controller_manager 同时开一个 tty 在 Linux 上是允许的（默认不排他），结果就是两套
  // 各发各的帧、互相踩数据 —— 2026-10-05 真机踩过（②的 launch 没关就起③，看着像"发轨迹不动"）。
  // 设了 TIOCEXCL 之后，第二家 open() 会直接 EBUSY 失败，报错清楚得多。
  // 失败不致命（个别设备不支持），只是少了这层保护。
  if (::ioctl(fd_, TIOCEXCL) != 0) {
    // 什么都不做：不阻断正常使用
  }

  struct termios tio {};
  if (::tcgetattr(fd_, &tio) != 0) {
    const std::string msg = errno_text("tcgetattr 失败");
    close();
    throw std::runtime_error(msg);
  }
  ::cfmakeraw(&tio);                      // 8N1、无回显、无行缓冲
  tio.c_cflag |= (CLOCAL | CREAD);        // 忽略调制解调器控制线、允许接收
  tio.c_cflag &= ~CRTSCTS;                // 无硬件流控
  tio.c_cc[VMIN] = 0;                     // 非阻塞读：读不到就立刻返回
  tio.c_cc[VTIME] = 0;
  if (::cfsetspeed(&tio, baud_to_speed(baud_)) != 0) {
    const std::string msg = errno_text("设置波特率失败");
    close();
    throw std::runtime_error(msg);
  }
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    const std::string msg = errno_text("tcsetattr 失败");
    close();
    throw std::runtime_error(msg);
  }
  ::tcflush(fd_, TCIFLUSH);               // 丢掉开设备时可能已经堆着的陈旧字节
}

void SerialPort::close()
{
  if (fd_ < 0) {return;}
  const int fd = fd_;
  fd_ = -1;
  ::close(fd);
}

void SerialPort::write(const uint8_t * data, std::size_t len)
{
  if (fd_ < 0) {throw std::runtime_error("串口没开，不能写");}
  stats_.tx_calls += 1;
  std::size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, data + written, len - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;   // 非阻塞口写满时要重试；这里帧很小（30B），不会真的卡住
    }
    throw std::runtime_error(errno_text("写串口失败"));
  }
  stats_.tx_bytes += static_cast<uint64_t>(written);
}

std::size_t SerialPort::bytes_available() const
{
  if (fd_ < 0) {return 0;}
  int pending = 0;
  if (::ioctl(fd_, FIONREAD, &pending) != 0) {return 0;}
  return pending > 0 ? static_cast<std::size_t>(pending) : 0u;
}

std::size_t SerialPort::read_available(std::vector<uint8_t> & out)
{
  if (fd_ < 0) {throw std::runtime_error("串口没开，不能读");}
  // 一次读干净：先问内核还有多少，再读那么多（高速循环里别一字节一字节抠）
  std::size_t total = 0;
  while (true) {
    const std::size_t pending = bytes_available();
    if (pending == 0) {break;}
    const std::size_t old = out.size();
    out.resize(old + pending);
    const ssize_t n = ::read(fd_, out.data() + old, pending);
    if (n > 0) {
      out.resize(old + static_cast<std::size_t>(n));
      total += static_cast<std::size_t>(n);
      stats_.rx_calls += 1;
      stats_.rx_bytes += static_cast<uint64_t>(n);
      continue;
    }
    out.resize(old);
    if (n < 0 && errno == EINTR) {continue;}
    break;   // EAGAIN：内核说没了
  }
  return total;
}

void SerialPort::flush_input()
{
  if (fd_ < 0) {return;}
  ::tcflush(fd_, TCIFLUSH);
}

}  // namespace dm_armx_hardware
