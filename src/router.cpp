#include "router.hpp"

Usb2CanRouter::Usb2CanRouter(const std::vector<Usb2CanDevice*>& devs) : devs_(devs) {}

Usb2CanRouter::~Usb2CanRouter() { stop(); }

void Usb2CanRouter::attach_motor(RobStrideMotor* m) {
  std::lock_guard<std::mutex> lk(mu_);
  Key k;
  // k.dev_idx = m->bus().dev_idx;
  k.dev = m->bus().dev;
  // k.channel = m->bus().channel;
  k.channel = (uint8_t)m->bus().channel;
  // 注意：状态帧分发使用 bits[15:8] = motor_id
  k.motor_id_bits15_8 = m->motor_id();
  map_[k] = m;
  std::cout << "[ATTACH] dev_ptr=" << k.dev
            << " channel=" << int(k.channel)
            << " motorID=" << int(k.motor_id_bits15_8)
            << "\n";
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

void Usb2CanRouter::rx_loop(int dev_vec_idx) {
  Usb2CanFrame f;
  while (running_.load())
  {
    if (!devs_[dev_vec_idx]->recv(f, 20000))
      continue; // 20ms

    uint8_t motor_id_bits15_8 = (f.eid >> 8) & 0xFF;
    uint8_t comm = (f.eid >> 24) & 0x1F;

    RobStrideMotor *m = nullptr;
    {
      std::lock_guard<std::mutex> lk(mu_);
      Key k{devs_[dev_vec_idx], (uint8_t)f.channel, motor_id_bits15_8};
      auto it = map_.find(k);
      if (it != map_.end())
        m = it->second;
    }
    if (m)
      m->on_frame(f);
    static int dbg = 0;
    if (dbg++ < 50)
    {
      std::cout << "[RX] dev_ptr==" << dev_vec_idx
                << " ch=" << int(f.channel)
                << " eid=0x" << std::hex << f.eid << std::dec
                << " comm=" << int(comm)
                << " mid=" << int(motor_id_bits15_8)
                << (m ? "  -> DISPATCH\n" : "  -> NO_MATCH\n");
    }
  }
}
