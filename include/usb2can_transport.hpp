#pragma once
#include <string>
#include <array>
#include <stdexcept>
#include <cstdint>

// 这里引用你的 Tangair SDK
#include "usb_can.h"

struct Usb2CanFrame {
  uint8_t channel = 0; // 1 or 2
  uint32_t eid = 0;
  std::array<uint8_t,8> data{};
  uint8_t dlc = 8;
};

class Usb2CanDevice {
public:
  Usb2CanDevice() = default;
  explicit Usb2CanDevice(const std::string& dev_path) { open(dev_path); }

  void open(const std::string& dev_path) {
    path_ = dev_path;
    fd_ = openUSBCAN(dev_path.c_str());
    if (fd_ < 0) throw std::runtime_error("openUSBCAN failed: " + dev_path);
  }

  ~Usb2CanDevice() {
    if (fd_ >= 0) closeUSBCAN(fd_);
  }

  const std::string& path() const { return path_; }
  int fd() const { return fd_; }

  void send(uint8_t channel, uint32_t eid, const std::array<uint8_t,8>& data, uint8_t dlc=8) {
    FrameInfo info{};
    info.canID = eid;
    info.frameType = EXTENDED;
    info.dataLength = dlc;

    uint8_t raw[8];
    for (int i=0;i<8;i++) raw[i] = data[i];
    (void)sendUSBCAN(fd_, channel, &info, raw);
  }

  bool recv(Usb2CanFrame& out, int timeout_us) {
    FrameInfo info_rx{};
    uint8_t channel = 0;
    uint8_t raw[8] = {0};

    int ret = readUSBCAN(fd_, &channel, &info_rx, raw, timeout_us);
    if (ret == -1) return false;

    out.channel = channel;
    out.eid = info_rx.canID;
    out.dlc = info_rx.dataLength;
    for (int i=0;i<8;i++) out.data[i] = raw[i];
    return true;
  }

private:
  std::string path_;
  int fd_ = -1;
};
