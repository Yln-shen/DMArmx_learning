// dm_serial.hpp —— 串口（只做"字节进、字节出"）。
//
// 边界：不解析帧、不判安全、不 sleep、不打屏、不知道电机是什么。
// 分成 `SerialIo` 接口 + `SerialPort` 实现，是为了**测试能塞假串口**（内存字节流），
// 不必真开设备 —— 与 Python 侧给 `bus.ser` 注入假对象是同一个套路。
// 接口 + 实现 + 非阻塞
#ifndef MOTOR_DRIVER_HARDWARE__DM_SERIAL_HPP_
#define MOTOR_DRIVER_HARDWARE__DM_SERIAL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dm_armx_hardware
{

struct SerialStats
{
  uint64_t tx_bytes = 0;
  uint64_t tx_calls = 0;
  uint64_t rx_bytes = 0;
  uint64_t rx_calls = 0;
};

class SerialIo
{
public:
  virtual ~SerialIo() = default;
  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual void write(const uint8_t * data, std::size_t len) = 0;
  // 非阻塞：把"已经到的"字节追加进 out，返回读到的字节数（可能是 0）
  virtual std::size_t read_available(std::vector<uint8_t> & out) = 0;
  virtual std::size_t bytes_available() const = 0;   // FIONREAD
  virtual void flush_input() = 0;
  virtual const SerialStats & stats() const = 0;
};

class SerialPort : public SerialIo
{
public:
  explicit SerialPort(std::string device, int baud = 921600);
  ~SerialPort() override;
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void open() override;
  void close() override;
  bool is_open() const override {return fd_ >= 0;}
  void write(const uint8_t * data, std::size_t len) override;
  std::size_t read_available(std::vector<uint8_t> & out) override;
  std::size_t bytes_available() const override;
  void flush_input() override;
  const SerialStats & stats() const override {return stats_;}

  const std::string & device() const {return device_;}
  int baud() const {return baud_;}

private:
  std::string device_;
  int baud_;
  int fd_ = -1;
  SerialStats stats_;
};

}  // namespace dm_armx_hardware

#endif  // MOTOR_DRIVER_HARDWARE__DM_SERIAL_HPP_
