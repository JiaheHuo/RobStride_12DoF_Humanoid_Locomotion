#include "router.hpp"

Usb2CanRouter::Usb2CanRouter(const std::vector<Usb2CanDevice*>& devs) : devs_(devs) {}

Usb2CanRouter::~Usb2CanRouter() { stop(); }

void Usb2CanRouter::attach_motor(RobStrideMotor* m) {
  std::lock_guard<std::mutex> lk(mu_);
  Key k;
  k.dev_idx = m->bus().dev_idx;
  k.channel = m->bus().channel;
  // 注意：状态帧分发使用 bits[15:8] = motor_id
  k.motor_id_bits15_8 = m->motor_id();
  map_[k] = m;
}

void Usb2CanRouter::start() {
  running_.store(true);
  threads_.clear();
  threads_.reserve(devs_.size());
  for (int i=0;i<(int)devs_.size();++i) {
    threads_.emplace_back([this,i]{ rx_loop(i); });
  }
}

void Usb2CanRouter::stop() {
  running_.store(false);
  for (auto& t : threads_) {
    if (t.joinable()) t.join();
  }
  threads_.clear();
}

void Usb2CanRouter::rx_loop(int dev_idx) {
  Usb2CanFrame f;
  while (running_.load()) {
    // 用短一点 timeout，方便退出
    if (!devs_[dev_idx]->recv(f, 20000)) continue; // 20ms

    uint8_t motor_id_bits15_8 = (f.eid >> 8) & 0xFF;

    RobStrideMotor* m = nullptr;
    {
      std::lock_guard<std::mutex> lk(mu_);
      Key k{dev_idx, f.channel, motor_id_bits15_8};
      auto it = map_.find(k);
      if (it != map_.end()) m = it->second;
    }
    if (m) m->on_frame(f);
  }
}
